TARGET = Proteus
export TARGET


GITBINARY := git
FIRMWARE_FEH_REPO := https://code.osu.edu/fehelectronics/proteus_software/fehproteusfirmware
FIRMWARE_BRIAN_REPO := https://github.com/Powerlated/fehproteusfirmware
FIRMWAREREPO := fehproteusfirmware

all: | fehproteusfirmware
	cd $(FIRMWAREREPO) && git pull feh master
	cd $(FIRMWAREREPO) && git pull origin master
	$(MAKE) -C $(FIRMWAREREPO) all
	$(MAKE) -C $(FIRMWAREREPO) deploy

fehproteusfirmware:
	git init
	git submodule add $(FIRMWARE_BRIAN_REPO)
	cd $(FIRMWAREREPO) && git remote add feh $(FIRMWARE_FEH_REPO)
	cd $(FIRMWAREREPO) && git config pull.rebase false

deploy:
	$(MAKE) -C $(FIRMWAREREPO) deploy

clean:
	$(MAKE) -C $(FIRMWAREREPO) clean

run:
	$(MAKE) -C $(FIRMWAREREPO) run