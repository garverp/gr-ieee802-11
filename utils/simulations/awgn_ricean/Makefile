SHELL=/bin/bash

MESSAGES=100
PCAPFILES=$(shell python -c 'import itertools; import numpy as np; print(" ".join(["results/sim_%d_%d_%.1f_.pcap" % x for x in itertools.product(range(20), range(2,3), np.arange(0, 30,.5))]))')

all: pdr.pdf

pdr.pdf: plot.py results/all.csv
	python ./plot.py

sim_ricean.py: sim_ricean.grc
	grcc -d . sim_ricean.grc

results/all.csv: $(PCAPFILES)
	./parse.sh

%.pcap: sim_ricean.py
	@echo $@
	mkdir -p results
	$(eval REPETITION=$(shell python -c "print \"$@\".split(\"_\")[1]"))
	$(eval ENCODING=$(shell python -c "print \"$@\".split(\"_\")[2]"))
	$(eval SNR=$(shell python -c "print \"$@\".split(\"_\")[3]"))
	@echo REPETITION=$(REPETITION)
	@echo ENCODING=$(ENCODING)
	@echo SNR=$(SNR)
	./sim_ricean.py --messages=$(MESSAGES) --encoding=$(ENCODING) --snr=$(SNR) --repetition=$(REPETITION)

