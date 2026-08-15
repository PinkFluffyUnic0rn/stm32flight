## Copyright (C) 2026 qwerty
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} furier (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: qwerty <qwerty@oO>
## Created: 2026-07-25

function retval = furier (data, p)
  h = hann(4000);

  #for i = 16000 : 24000
    s = p;
    e = p + 3999;

    xw = data(s:e) .* h;

    rv = abs(fft(xw));

    retval = rv(1:1000);

  #  retval(i,:) = rv(1:2000);
 # endfor
endfunction
