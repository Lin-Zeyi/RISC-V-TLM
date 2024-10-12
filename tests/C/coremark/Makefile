TARGET   = coremark

TARGET_ARCH = riscv64

CC       = riscv64-unknown-elf-gcc

ifndef $(ITERATIONS)
ITERATIONS=1000
endif

# compiling flags here
CFLAGS = -Wall -I. -O3 -static -march=rv64imac_zicsr -mabi=lp64 --specs=nosys.specs -ggdb -DFLAGS_STR
CFLAGS += -DITERATIONS=$(ITERATIONS)


LINKER   = riscv64-unknown-elf-gcc
# linking flags here
LDFLAGS   = -I. -static -g
LIBS   = $(EXTRA_LIBS)


# change these to proper directories where each file should be
SRCDIR   = .
OBJDIR   = ./build
BINDIR   = $(OBJDIR)
INCDIR = -I.
LIBDIR = -L.


SOURCES  := $(wildcard $(SRCDIR)/*.c)
INCLUDES := $(wildcard $(INCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

rm       = rm -f

all: prepare $(BINDIR)/$(TARGET)

$(BINDIR)/$(TARGET): $(OBJECTS)
	$(LINKER) $(CFLAGS) $(LDFLAGS) $(LIBS) $(LIBDIR) $(OBJECTS) -o $@
	riscv64-unknown-elf-objdump -d $@ > dump
	riscv64-unknown-elf-objcopy -Oihex $@ $(TARGET).hex
	@echo "Linking complete!"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.c
	@echo "Compiling "$<" ..."
	$(CC) $(CFLAGS) $(INCDIR) -c $< -o $@
	@echo "Done!"

prepare:
	mkdir -p $(OBJDIR)

.PHONY: clean prepare
clean:
	@$(rm) $(OBJECTS) *.hex dump
	@echo "Cleanup complete!"

.PHONY: remove
remove: clean
	@$(rm) $(BINDIR)/$(TARGET)
	@echo "Executable removed!"

all: $(BINDIR)/$(TARGET)
