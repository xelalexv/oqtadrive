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

package format

import (
	"fmt"
	"io"

	"github.com/xelalexv/oqtadrive/pkg/microdrive/base"
	"github.com/xelalexv/oqtadrive/pkg/microdrive/format/z80"
	"github.com/xelalexv/oqtadrive/pkg/util"
)

// Z80 is a format for loading Z80 snapshots. It is an asymmetrical format in
// the sense that it reads Z80 snapshots, but writes MDRs. The SNA format is,
// while being a format on its own, essentially an uncompressed V1 Z80 snapshot.
// It is therefore handled by Z80.
type Z80 struct {
	sna bool
}

//
func NewZ80(sna bool) *Z80 {
	return &Z80{sna}
}

//
func (z *Z80) Read(in io.Reader, strict, repair bool,
	p util.Params) (cart *base.Cartridge, err error) {

	defer func() {
		if e := recover(); e != nil {
			cart = nil
			err = fmt.Errorf("unrecoverable error during snapshot conversion: %v", e)
		}
	}()

	name, _ := p.GetString("name")
	launcher, _ := p.GetString("launcher")

	cart, err = z80.LoadZ80(in, name, launcher, z.sna)
	if err != nil {
		return nil, err
	}

	if repair {
		RepairOrder(cart)
	}

	cart.SetModified(false)
	cart.SeekToStart()
	cart.RewindAccessIx(true)

	return cart, nil
}

//
func (z *Z80) Write(cart *base.Cartridge, out io.Writer, p util.Params) error {
	return NewMDR().Write(cart, out, p)
}
