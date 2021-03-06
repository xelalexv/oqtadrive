/*
   OqtaDrive - Sinclair Microdrive emulator
   Copyright (c) 2021, Alexander Vollschwitz

   This file is part of OqtaDrive.

   OqtaDrive is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   OqtaDrive is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with OqtaDrive. If not, see <http://www.gnu.org/licenses/>.
*/

package control

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gorilla/mux"
	log "github.com/sirupsen/logrus"

	"github.com/xelalexv/oqtadrive/pkg/daemon"
	"github.com/xelalexv/oqtadrive/pkg/repo"
)

//
type APIServer interface {
	Serve() error
	Stop() error
}

//
func NewAPIServer(addr, repo string, d *daemon.Daemon) APIServer {
	return &api{address: addr, repository: repo, daemon: d}
}

//
type api struct {
	//
	address    string
	repository string
	//
	daemon *daemon.Daemon
	server *http.Server
	index  *repo.Index
	//
	longPollQueue chan chan *Change
	forceNotify   chan bool
	stopped       bool
}

//
func (a *api) Serve() error {

	router := mux.NewRouter().StrictSlash(true)

	addRoute(router, "status", "GET", "/status", a.status)
	addRoute(router, "watch", "GET", "/watch", a.watch)
	addRoute(router, "ls", "GET", "/list", a.list)
	addRoute(router, "load", "PUT", "/drive/{drive:[1-8]}", a.load)
	addRoute(router, "unload", "GET", "/drive/{drive:[1-8]}/unload", a.unload)
	addRoute(router, "save", "GET", "/drive/{drive:[1-8]}", a.save)
	addRoute(router, "dump", "GET", "/drive/{drive:[1-8]}/dump", a.dump)
	addRoute(router, "map", "GET", "/map", a.getDriveMap)
	addRoute(router, "map", "PUT", "/map", a.setDriveMap)
	addRoute(router, "drivels", "GET", "/drive/{drive:[1-8]}/list", a.driveList)
	addRoute(router, "resync", "PUT", "/resync", a.resync)
	addRoute(router, "config", "GET", "/config", a.getConfig)
	addRoute(router, "config", "PUT", "/config", a.setConfig)
	addRoute(router, "search", "GET", "/search", a.search)
	addRoute(router, "upgrade", "POST", "/upgrade", a.upgrade)
	addRoute(router, "version", "GET", "/version", a.version)

	router.PathPrefix("/").Handler(
		requestLogger(http.FileServer(http.Dir("./ui/web/")), "webui"))

	if a.repository != "" {
		var err error
		a.index, err = repo.NewIndex("repo.index", a.repository)
		if err != nil {
			log.Errorf("failed to open/create index: %v", err)
		} else {
			go func() {
				if err := a.index.Start(); err != nil {
					log.Errorf("error starting index: %v", err)
				}
			}()
		}
	}

	addr := a.address
	if len(strings.Split(addr, ":")) < 2 {
		addr = fmt.Sprintf("%s:8888", a.address)
	}

	log.Infof("OqtaDrive API starts listening on %s", addr)
	a.server = &http.Server{Addr: addr, Handler: router}

	a.longPollQueue = make(chan chan *Change)
	a.forceNotify = make(chan bool, 1)
	go a.watchDaemon()

	err := a.server.ListenAndServe()
	if err != nil && err != http.ErrServerClosed {
		return err
	}
	return nil
}

//
func (a *api) Stop() error {

	a.stopped = true

	if a.index != nil {
		log.Info("index stopping...")
		a.index.Stop()
		a.index = nil
		log.Info("index stopped")
	}

	a.discardLongPollClients()

	if a.server != nil {
		log.Info("API server stopping...")
		err := a.server.Shutdown(context.Background())
		a.server = nil
		return err
	}

	return nil
}

//
func (a *api) getCartridges() []*Cartridge {

	ret := make([]*Cartridge, daemon.DriveCount)

	for drive := 1; drive <= daemon.DriveCount; drive++ {

		c := &Cartridge{Status: a.daemon.GetStatus(drive)}

		if c.Status == daemon.StatusIdle {
			if cart, ok := a.daemon.GetCartridge(drive); cart != nil {
				c.fill(cart)
				cart.Unlock()
			} else if !ok {
				c.Status = daemon.StatusBusy
			}
		}

		ret[drive-1] = c
	}

	return ret
}

//
func addRoute(r *mux.Router, name, method, pattern string,
	handler http.HandlerFunc) {
	r.Methods(method).
		Path(pattern).
		Name(name).
		Handler(requestLogger(handler, name))
}

//
func requestLogger(inner http.Handler, name string) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		log.WithFields(log.Fields{
			"remote": r.RemoteAddr,
			"method": r.Method,
			"path":   r.RequestURI,
		}).Debugf("API BEGIN | %s", name)

		start := time.Now()
		inner.ServeHTTP(w, r)

		log.WithFields(log.Fields{
			"remote":   r.RemoteAddr,
			"method":   r.Method,
			"path":     r.RequestURI,
			"duration": time.Since(start),
		}).Debugf("API END   | %s", name)
	})
}

//
func getDrive(w http.ResponseWriter, req *http.Request) int {
	vars := mux.Vars(req)
	drive, err := strconv.Atoi(vars["drive"])
	if handleError(err, http.StatusUnprocessableEntity, w) {
		return -1
	}
	return drive
}

//
func getRef(req *http.Request) (string, error) {
	if !isFlagSet(req, "ref") {
		return "", nil
	}
	buf := new(strings.Builder)
	if _, err := io.Copy(buf, io.LimitReader(req.Body, 1024)); err != nil {
		return "", err
	}
	return buf.String(), nil
}

//
func isFlagSet(req *http.Request, flag string) bool {
	return getArg(req, flag) == "true"
}

//
func getArg(req *http.Request, arg string) string {
	return req.URL.Query().Get(arg)
}

//
func getIntArg(req *http.Request, arg string, def int) (int, error) {
	if val := getArg(req, arg); val == "" {
		if def > -1 {
			return def, nil
		}
		return -1, fmt.Errorf("int argument not set: %s", arg)
	} else if ret, err := strconv.Atoi(val); err != nil {
		return def, err
	} else {
		return ret, nil
	}
}

//
func setHeaders(h http.Header, json bool) {
	if json {
		h.Set("Content-Type", "application/json; charset=UTF-8")
	} else {
		h.Set("Content-Type", "text/plain; charset=UTF-8")
	}
}

//
func handleError(e error, statusCode int, w http.ResponseWriter) bool {

	if e == nil {
		return false
	}

	log.Errorf("%v", e)

	setHeaders(w.Header(), false)
	w.WriteHeader(statusCode)
	if _, err := w.Write([]byte(fmt.Sprintf("%v\n", e))); err != nil {
		log.Errorf("problem writing error: %v", err)
	}

	return true
}

//
func sendReply(body []byte, statusCode int, w http.ResponseWriter) {
	setHeaders(w.Header(), false)
	w.WriteHeader(statusCode)
	if _, err := fmt.Fprintf(w, "%s\n", body); err != nil {
		log.Errorf("problem sending reply: %v", err)
	}
}

//
func sendStreamReply(r io.Reader, statusCode int, w http.ResponseWriter) {
	setHeaders(w.Header(), false)
	w.WriteHeader(statusCode)
	if _, err := io.Copy(w, r); err != nil {
		log.Errorf("problem sending reply: %v", err)
	}
}

//
func sendJSONReply(obj interface{}, statusCode int, w http.ResponseWriter) {
	setHeaders(w.Header(), true)
	w.WriteHeader(statusCode)
	if err := json.NewEncoder(w).Encode(obj); err != nil {
		log.Errorf("problem writing error: %v", err)
	}
}

// FIXME: make more tolerant
func wantsJSON(req *http.Request) bool {
	return req.Header.Get("Content-Type") == "application/json"
}
