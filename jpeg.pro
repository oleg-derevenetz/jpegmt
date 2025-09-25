include(Libs.pri) # for HELPER_DIR

TEMPLATE = subdirs

SUBDIRS += lib/Jpeg

!no-helper : SUBDIRS += $${HELPER_DIR}

!no-test {
  no-helper : SUBDIRS += $${HELPER_DIR}
  SUBDIRS += test
  test.depends = lib/Jpeg $${HELPER_DIR}
}

