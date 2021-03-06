/*
   OqtaDrive - Sinclair Microdrive emulator
   Copyright (c) 2022, Alexander Vollschwitz

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

package util

//
type Params map[string]interface{}

//
func (p Params) GetString(key string) (string, bool) {
	if p != nil {
		if v, ok := p[key]; ok && v != nil {
			if str, ok := v.(string); ok {
				return str, true
			}
		}
	}
	return "", false
}

//
func (p Params) GetInt(key string) (int, bool) {
	if p != nil {
		if v, ok := p[key]; ok && v != nil {
			if i, ok := v.(int); ok {
				return i, true
			}
		}
	}
	return -1, false
}

//
func (p Params) GetBool(key string) (bool, bool) {
	if p != nil {
		if v, ok := p[key]; ok && v != nil {
			if b, ok := v.(bool); ok {
				return b, true
			}
		}
	}
	return false, false
}
