# coinop-badge

PCB badge inspired by the classic coin-op game Galaga

https://hackaday.io/project/159302/

# KiCAD Library dependencies

Install the KiCad libraries:
` sudo apt-get install kicad-library

Install the [https://github.com/MacroFab/EDALibraries](MacroFab EDALibraries):
` cd ~/compile
` git clone https://github.com/MacroFab/EDALibraries
` cd /usr/share/kicad/modules 
` sudo ln -s ~/compile/EDALibraries/KiCad/Footprints/* .
` cd /usr/share/kicad/library
` sudo ln -s ~/compile/EDALibraries/KiCad/Schematics/* .

