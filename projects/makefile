# Build all the projects in the directory

subdirs=$(shell find . -mindepth 1 -maxdepth 1 -type d)
all:
	@echo "Building all the projects in the directory"
	@for d in $(subdirs); do make -C $$d || exit 1; done

clean:
	@for d in $(subdirs); do make -C $$d $@; done
	
cpp-projects : 
	make -C cpp
	make -C cpp2
	make -C cpp3
	make -C cpp4
