SCHEMATIC = room-board.sch
GEDA_DIR = ~/geda/bin/
# GEDA_DIR = 

all: check

check: 
	$(GEDA_DIR)gnetlist -g drc2 $(SCHEMATIC) -o drc_output.txt
	cat drc_output.txt

gsch2pcb:
	$(GEDA_DIR)gsch2pcb project

pcb:
	$(GEDA_DIR)pcb board.pcb&

schematic:
	$(GEDA_DIR)gschem $(SCHEMATIC)&

bom:
	$(GEDA_DIR)gnetlist -g bom -o bom.txt $(SCHEMATIC)
#	gnetlist -g bom2 -o bom.txt $(SCHEMATIC)
#	gnetlist -g partslist1 -o bom.txt $(SCHEMATIC)
#	gnetlist -g partslist2 -o bom.txt $(SCHEMATIC)
#	gnetlist -g partslist3 -o bom.txt $(SCHEMATIC)
	cat bom.txt
#	gattrib -i $(SCHEMATIC)&

renum:
#	refdes_renum $(SCHEMATIC)
	$(GEDA_DIR)refdes_renum --force $(SCHEMATIC)
.SILENT: check

clean:
	rm -f *~ *.bak* *.gbr *.cnc *- *.txt *.ps *.pdf *.new.pcb
