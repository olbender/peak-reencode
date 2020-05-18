# Copyright (C) 2020  Christian Berger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Part to build peak_reencode.
FROM alpine:3.11.3 as builder
MAINTAINER Christian Berger "christian.berger@gu.se"
RUN apk update && \
    apk --no-cache add \
        cmake \
        g++ \
        make \
        upx
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/peak_reencode-dest .. && \
    make && make install && upx -9 /tmp/peak_reencode-dest/bin/peak_reencode


# Part to deploy peak_reencode.
FROM alpine:3.11.3
MAINTAINER Christian Berger "christian.berger@gu.se"

WORKDIR /usr/bin
COPY --from=builder /tmp/peak_reencode-dest/bin/peak_reencode .
CMD ["/usr/bin/peak_reencode"]
