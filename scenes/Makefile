.PHONY : all

#n.b. the '-y' sets autoexec scripts to 'on' so that driver expressions will work
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	BLENDER = /Applications/Blender.app/Contents/MacOS/Blender -y
else ifeq ($(UNAME_S),Linux)
	BLENDER = ../../blender-4.5.2-linux-x64/blender -y
else
	BLENDER = blender
endif

DIST=../dist

all : $(DIST)/lightbound.pnct

# Lightbound: procedural placeholders + Blender props from dist/objects.blend
$(DIST)/lightbound-props.pnct : ../dist/objects.blend export-lightbound-props.py
	$(BLENDER) --background --python export-lightbound-props.py -- '../dist/objects.blend' '$@'

$(DIST)/lightbound-base.pnct : generate-lightbound-meshes.py
	python3 generate-lightbound-meshes.py '$@'

$(DIST)/lightbound.pnct : $(DIST)/lightbound-base.pnct $(DIST)/lightbound-props.pnct merge-pnct.py
	python3 merge-pnct.py '$@' $(DIST)/lightbound-base.pnct $(DIST)/lightbound-props.pnct
