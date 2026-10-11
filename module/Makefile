# SPDX-License-Identifier: GPL-2.0
# Out-of-tree build for local testing; DKMS uses dkms.conf instead.
KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

.PHONY: all clean
