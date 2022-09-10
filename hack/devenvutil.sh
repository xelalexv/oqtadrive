#!/usr/bin/env bash

#
#   OqtaDrive - Sinclair Microdrive emulator
#   Copyright (c) 2021, Alexander Vollschwitz
#
#   This file is part of OqtaDrive.
#
#   OqtaDrive is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   OqtaDrive is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with OqtaDrive. If not, see <http://www.gnu.org/licenses/>.
#

#
# Note: All variables defined in Makefile can be directly accessed here.
#

# shellcheck disable=SC2034
{
# formatting
BLD="\e[1m"
DIM="\e[2m"
ITL="\e[3m"
NRM="\e[0m"
OK="\e[01;32m"
ERR="\e[01;31m"
}

#
#
#
function synopsis {

    files=()

    command -v gawk > /dev/null || echo \
        "Note: proper help display requires gawk! (e.g. sudo apt install gawk)"

    for file in ${MAKEFILE_LIST}; do
        if [[ "$(basename "${file}")" == "Makefile" ]]; then
            files+=( "../${file}" )
        fi
    done

    echo -e "\n${BLD}TARGETS${NRM}"
    print_sorted_help "$(cat "${files[@]}" \
        | gawk '{FS=":"}
            /^[a-zA-Z0-9][-a-zA-Z0-9_\.]+:{1,2}[-a-zA-Z0-9_\. \/]*$/{f=1; printf "\n${ITL}${BLD}%s${NRM}\n", $1; next}
            /^[^#].*$/{f=0} f' \
        | tr -d '#')"

    echo -e "\n${BLD}NOTES${NRM}\n"

    # .makerc settings
    print_formatted_help "$(cat "${files[@]}" \
        | gawk '/^## makerc$/{f=1; next} /^[^#].*$/{f=0} /^$/{f=0} f' \
        | tr -d '#')"
    echo

    # env settings
    print_formatted_help "$(cat "${files[@]}" \
        | gawk '/^## env$/{f=1; next} /^[^#].*$/{f=0} /^$/{f=0} f' \
        | tr -d '#')"

    # other notes
    print_formatted_help "$(cat "${files[@]}" \
        | gawk '/^##$/{f=1; printf "%s", $0; next} /^[^#].*$/{f=0} /^$/{f=0} f' \
        | tr -d '#')"
    echo
}

#
# $1    help text
#
function print_sorted_help {
    print_formatted_help "$1" \
        | gawk 'BEGIN{print "\0"}
            /^$/{printf "\0"} {print $0}' \
        | sort -z \
        | tr -d '\000' \
        | tail -n+2
}

#
# $1    help text
#
function print_formatted_help {
    echo -e "$(apply_shell_expansion "$1")" | uniq
}

#
# $1    string to expand
#
function apply_shell_expansion {
    declare data="$1"
    declare delimiter="__apply_shell_expansion_delimiter__"
    declare command="cat <<${delimiter}"$'\n'"${data}"$'\n'"${delimiter}"
    eval "${command}"
}

#
# build command binary
#
# $1    command
# $2    target OS
# $3    target architecture; omit for `amd64`
# $4    `keep` for keeping the binary, not just the archive; requires $3
#
function build_binary {

    local arch="amd64"
    [[ -z "$3" ]] || arch="$3"

    local suffix
    [[ "$2" != "windows" ]] || suffix=".exe"

    local binary="${BINARIES}/$1${suffix}"

    local extra_env
    [[ "${arch}" != "arm" ]] || extra_env="-e GOARM=6"

    echo -e "\nbuilding ${binary} for $2/${arch}"

    # shellcheck disable=SC2086
    docker run --rm --user "$(id -u):$(id -g)" \
        -v "${ROOT}/${BINARIES}:/go/bin" ${CACHE_VOLS} \
        -v "${ROOT}:/go/src/${REPO}" -w "/go/src/${REPO}" \
        -e CGO_ENABLED=0 -e GOOS="$2" -e GOARCH="${arch}" ${extra_env} \
        "${GO_IMAGE}" bash -c \
            "go mod tidy && go build -v -tags netgo -installsuffix netgo -ldflags \
            \"-s -w -X codeberg.org/xelalexv/oqtadrive/pkg/util.OqtaDriveVersion=${OQTADRIVE_VERSION}\" \
            -o \"${binary}\" \"./cmd/$1/\""

    local specifier="_${OQTADRIVE_RELEASE}_$2_${arch}"
    zip -j "../${binary}${specifier}.zip" "../${binary}"

    if [[ "$4" == "keep" ]]; then
        mv "../${binary}" "../${binary}${specifier}"
    else
        rm -f "../${binary}"
    fi
}

#
# $...  names of JavaScript files to include (without path); order matters
#
function minify_js {

    local oqta="${UI_BASE}/js/oqta.js"

    pushd "${UI_BASE}/js/oqta" > /dev/null || return 1
    cat "$@" > "${oqta}"
    popd > /dev/null || return 1

    docker run --rm --user "$(id -u):$(id -g)" -v "${UI_BASE}/js:/data" -w /data \
        "${JSMINIFY_IMAGE}" minify -o oqta.min.js oqta.js

    local hash
    hash="$(sha1sum "${UI_BASE}/js/oqta.min.js" | head -c 6)"
    # shellcheck disable=SC2086
    rm -f ${UI_BASE}/js/oqta.min.*.js
    mv "${UI_BASE}/js/oqta.min.js" "${UI_BASE}/js/oqta.min.${hash}.js"
    sed -Ei "s:/oqta.min.[[:alnum:]]+.js:/oqta.min.${hash}.js:g" "${UI_BASE}/index.html"
    rm -f "${oqta}"
}

#
#
#
function download_oqtactl {

    local arch
    arch="$(get_architecture)" || return 1

    local os
    os="$(get_os "${arch}")" || return 1

    local marker="${os,,}_${arch}"
    local url

    if [[ -z "${BUILD_URL}" ]]; then # get from release page
        url="$(get_asset_url "${marker}.zip")"

    else # get from custom build page
        url="$(curl -fsSL "${BUILD_URL}" \
            | grep "${marker}\.zip" \
            | cut -d '"' -f 8)" && url="${BUILD_URL}/${url}"
    fi

    if [[ -z "${url}" ]]; then
        echo -e \
            "\nNo download available for architecture '${arch}' on OS '${os}' in version '$(version_label "${VERSION}")'.\n" >&2
        return 1
    fi

    [[ ! -f "${OQTACTL}" ]] || mv -f "${OQTACTL}" "${OQTACTL}.bak"

    echo "  from ${url}"
    curl -fsSL "${url}" -o oqtactl.zip && \
        unzip -o oqtactl.zip && \
        rm oqtactl.zip && \
        chmod +x oqtactl && \
        mv oqtactl "${OQTACTL}" && \
        return 0

    echo "Download failed!" >&2
    mv -f "${OQTACTL}.bak" "${OQTACTL}"
    return 1
}

#
#
#
function download_ui {

    local url

    if [[ -z "${BUILD_URL}" ]]; then # get from release page
        url="$(get_asset_url ui.zip)"

    else # get from custom build page
        url="$(curl -fsSL "${BUILD_URL}" \
            | grep ui.zip \
            | cut -d '"' -f 8)" && url="${BUILD_URL}/${url}"
    fi

    if [[ -z "${url}" ]]; then
        echo -e "\nNo UI available in version '$(version_label "${VERSION}")'.\n" >&2
        return
    fi

    rm -rf "${ROOT}/ui.bak"
    [[ ! -d "${ROOT}/ui" ]] || mv -f "${ROOT}/ui" "${ROOT}/ui.bak"

    echo "  from ${url}"
    curl -fsSL "${url}" -o ui.zip && \
        rm -rf ui "${ROOT}/ui" && \
        unzip ui.zip && \
        rm ui.zip && \
        mv ui "${ROOT}/ui" && \
        return 0

    echo "Download failed!" >&2
    mv -f "${ROOT}/ui.bak" "${ROOT}/ui"
    return 1
}

#
#
#
function download_firmware {

    local url

    if [[ -z "${BUILD_URL}" ]]; then # get from repo
        [[ -n "${VERSION}" ]] || VERSION="$(get_latest_release)"
        url="${BASE_URL}/${VERSION}/arduino/oqtadrive.ino"

    else # get from custom build page
        url="${BUILD_URL}/oqtadrive.ino"
    fi

    [[ ! -f "${SKETCH}.org" ]] || mv -f "${SKETCH}.org" "${SKETCH}.org.bak"
    echo "  from ${url}"
    curl -fsSL -o "${SKETCH}.org" "${url}" && return 0

    echo "Download failed!" >&2
    mv -f "${SKETCH}.org.bak" "${SKETCH}.org"
    return 1
}

#
# $1    file name with path, relative to project root
# $2    `copy` to only create a copy with current version (optional), this is
#       for example needed when upgrading this shell script, since replacing it
#       while it is being used may cause errors; default is to replace file if
#       more current version is available
#
function assure_current {

    cd "${ROOT}" || { # when called from Makefile, we're in `hack`
        echo "corrupted project structure" >&2
        return 1
    }

    local ld
    if ! ld="$(to_utc "$1" ref)"; then
        ld="1970-01-01T01:00:00Z"
    fi

    # migration to codeberg: `since` not supported, get all with `path`,
    # but `limit` is ignored in that case, use first element from array
    local commits
    if ! commits="$(repo_api_call "commits?path=$1&sha=${BRANCH}&page=1")"; then
        echo "cannot determine latest commit" >&2
        return 1
    fi

    local len
    if ! len="$(echo "${commits}" | jq -r '. | length')" || [[ ${len} -eq 0 ]]; then
        echo "invalid commit reply from repo service" >&2
        return 1
    fi

    local rd
    if rd="$(echo "${commits}" | jq -r '.[0].commit.committer.date')" \
        && [[ -n "${rd}" ]]; then
        # Codeberg returns commit times in local time, convert to UTC
        rd="$(to_utc "${rd}")"
    else
        rd="$(to_utc)"
    fi

    if [[ ! "${rd}" > "${ld}" ]]; then
        return 0
    fi

    echo -e "$1 needs update (local date: ${ld}, remote date: ${rd}) ..."

    local dir
    dir="$(dirname "$1")"
    [[ -z "${dir}" ]] || mkdir -p "${dir}"

    local current="$1.current"
    if curl -fsSL -o "${current}" "${BASE_URL}/${BRANCH}/$1"; then
        # set to date of commit, but note that rd is in UTC, so we need to
        # convert to local time
        touch --date="$(date --date="${rd}" --iso-8601=seconds)" "${current}"
        chmod +x "${current}"
        [[ $# -gt 1 && "$2" == "copy" ]] || mv -f "${current}" "$1"
        echo "$1 updated"
    else
        echo "download of $1 failed" >&2
        rm -f "${current}"
        return 1
    fi
}

#
# $1    date string to convert; optional, current time when dropped
# $2    `ref` if $1 is a file path
#
function to_utc {
    local d
    if [[ -z "$1" ]]; then
        d="$(date --utc --iso-8601=seconds)"
    elif [[ $# -gt 1 && "$2" == "ref" ]]; then
        d="$(date --reference="$1" --utc --iso-8601=seconds 2>/dev/null)" \
            && [[ -n "${d}" ]] || return 1
    else
        d="$(date --date="$1" --utc --iso-8601=seconds)"
    fi
    echo -n "$(echo -n "${d}" | cut -d '+' -f 1)Z"
}

#
# $1    filter
#
function get_asset_url {

    # latest release; Codeberg does not accept `releases/latest`
    local path="releases?draft=false&pre-release=false&limit=1"
    local prefilter=".[0]"

    if [[ -n "${VERSION}" && "${VERSION}" != "latest" ]]; then
        path="releases/tags/${VERSION}"
        prefilter=""
    fi

    repo_api_call "${path}" 2>/dev/null \
        | jq -r "${prefilter}.assets[]
            | select(.name | contains(\"$1\"))
            | .browser_download_url"
}

#
#
#
function get_latest_release {
    repo_api_call "releases?draft=false&pre-release=false&limit=1" 2>/dev/null \
        | jq -r ".[0].name"
}

#
# $1    version; 'latest', '', or omit for latest version
#
function version_label {
    local l="latest"
    [[ -z "$1" ]] || l="$1"
    echo -n "${l}"
}

#
# $1    path
#
function repo_api_call {
    curl -fsSL -H "accept: application/json" \
        "https://codeberg.org/api/v1/repos/xelalexv/oqtadrive/$1"
}

#
# $...  actions, `install|remove|start|stop|enable|disable`
#
function manage_service {

    for a in "$@"; do

        case "${a}" in

            install)
                local extra_args
                if [[ -n "${REPO}" ]]; then
                    mkdir -p "${REPO}" || {
                        echo -e "\nCannot create repo folder '${REPO}'\n" >&2
                        return 1
                    }
                    extra_args+=" -r ${REPO}"
                fi
                curl -fsSL "${BASE_URL}/${BRANCH}/hack/oqtadrive.service" \
                    | sed -E -e "s;^ExecStart=.*$;ExecStart=${OQTACTL} serve -d ${PORT} -b ${BAUD_RATE} ${extra_args};g" \
                          -e "s;^WorkingDirectory=.*$;WorkingDirectory=${ROOT};g" \
                          -e "s;^User=.*$;User=${USER};g" \
                          -e "s;^Environment=.*$;Environment=LOG_LEVEL=info PORT=${PORT} BAUD_RATE=${BAUD_RATE} OLD_NANO=${OLD_NANO} RESET_PIN=${RESET_PIN} FQBN=${FQBN};g" \
                    | sudo tee /etc/systemd/system/oqtadrive.service > /dev/null
                sudo systemctl daemon-reload
                ;;

            remove)
                sudo rm --force /etc/systemd/system/oqtadrive.service
                ;;

            start|stop|status|enable|disable)
                [[ ! -f /etc/systemd/system/oqtadrive.service ]] || {
                    echo "running daemon service action: ${a}"
                    sudo systemctl "${a}" "${OQTADRIVE_SERVICE}"
                }
                ;;

            *)
                echo -e "\nUnknown service command: ${a}\n" >&2
                return 1
                ;;
        esac

        sleep 1
    done
}

#
#
#
function patch_avrdude {

    local avrdude
    avrdude="$(find ~/.arduino15/ -type f -name avrdude.org)"
    [[ -z "${avrdude}" ]] || {
        echo "avrdude already patched!"
        return 0
    }

    avrdude="$(find ~/.arduino15/ -type f -name avrdude)"

    [[ -n "${avrdude}" ]] || {
        echo "avrdude not installed!"
        return 1
    }

    mv "${avrdude}" "${avrdude}.org"

    local dir
    dir="$(dirname "${avrdude}")"

    local autoreset="${dir}/autoreset"
    curl -fsSL -o "${autoreset}" "${BASE_URL}/${BRANCH}/hack/autoreset"
    chmod +x "${autoreset}"

    cat <<EOF > "${avrdude}"
#!/bin/sh
sudo --preserve-env strace -o "|${autoreset}" -eioctl "${dir}/avrdude.org" \$@ 2>&1 | grep -vi "broken pipe"
EOF
    chmod +x "${avrdude}"
}

#
#
#
function unpatch_avrdude {

    local avrdude
    avrdude="$(find ~/.arduino15/ -type f -name avrdude.org)"
    [[ -n "${avrdude}" ]] || {
        echo "avrdude not patched!"
        return 0
    }

    local dir
    dir="$(dirname "${avrdude}")"

    mv --force "${avrdude}" "${dir}/avrdude"
    rm --force "${dir}/autoreset"
}

#
# $...  prerequisites
#
function prereqs {

    local missing

    for p in "$@"; do
        type "${p}" >/dev/null 2>&1 || {
            missing+=" ${p}"
        }
    done

    [[ -z "${missing}" ]] || {
        echo -e "\nYou need to install these dependencies:${missing}\n"
        return 1
    }
}

#
#
#
function sanity {
    local arch
    arch="$(get_architecture)" || return 1
    get_os "${arch}" > /dev/null || return 1
}

#
#
#
function get_architecture {

    local arch
    arch="$(uname -m)"

    case ${arch} in
        x86_64)
            echo -n "amd64"
            ;;
        x86|i386|i686)
            echo -n "386"
            ;;
        armv5*|armv6*|armv7*)
            echo -n "arm"
            ;;
        aarch64)
            echo -n "arm64"
            ;;
        *)
            echo -e "\nUnsupported architecture: ${arch}\n" >&2
            return 1
            ;;
    esac
}

#
# $1    architecture
#
function get_os {

    local os
    os="$(uname -s)"

    local err=1

    case ${os} in

        Linux)
            case $1 in
                amd64|amd|arm|386)
                    err=0
                    ;;
            esac
            ;;

        Darwin)
            case $1 in
                amd64|arm64)
                    err=0
                    ;;
            esac
            echo -e "\nWARNING: This has not been tested on MacOS yet!\n" >&2
            ;;

        *)
            echo -e "\n${os} not supported.\n" >&2
            return 1
            ;;
    esac

    [[ ${err} -eq 0 ]] || \
        {
            echo -e "\nArchitecture $1 not supported on ${os}.\n" >&2
            return 1
        }

    echo -n "${os}"
}

#
#
#

cd "$(dirname "$0")" || exit 1
"$@"
