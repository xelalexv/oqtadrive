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

package if1

import (
	"github.com/xelalexv/oqtadrive/pkg/microdrive/base"
	"github.com/xelalexv/oqtadrive/pkg/microdrive/client"
)

//
func NewCartridge() *base.Cartridge {
	impl := &cartridge{}
	cart := base.NewCartridge(client.IF1, SectorCount, impl)
	impl.Cartridge = cart
	cart.RewindAccessIx(false)
	return cart
}

//
type cartridge struct {
	*base.Cartridge
}

//
func (c *cartridge) FS() base.FileSystem {
	return newFs(c.Cartridge)
}
