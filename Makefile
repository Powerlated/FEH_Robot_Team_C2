TARGET = Proteus
export TARGET

GITBINARY := git
FEHURL := google.com
FIRMWAREREPO := fehproteusfirmware

all:
	$(MAKE) -C $(FIRMWAREREPO) all
	$(MAKE) -C $(FIRMWAREREPO) deploy

deploy:
	$(MAKE) -C $(FIRMWAREREPO) deploy

clean:
	$(MAKE) -C $(FIRMWAREREPO) clean

run:
	$(MAKE) -C $(FIRMWAREREPO) run