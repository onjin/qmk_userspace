CONSOLE_ENABLE = no
COMMAND_ENABLE = no
ORYX_ENABLE = yes
RGB_MATRIX_CUSTOM_KB = yes
TAP_DANCE_ENABLE = yes
SPACE_CADET_ENABLE = no
CAPS_WORD_ENABLE = yes
COMBO_ENABLE = yes
SRC += taipo.c

# Typing measurement build: qmk compile -kb zsa/voyager -km onjin -e KEYLOG=yes
# Logs every key press/release to the HID console, see keylog/README.md.
ifeq ($(strip $(KEYLOG)), yes)
    CONSOLE_ENABLE = yes
    OPT_DEFS += -DKEYLOG_ENABLE
    SRC += keylog.c
endif
