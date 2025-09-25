TEMPLATE = subdirs

SUBDIRS += lib/Jpeg

!no-test {
  SUBDIRS += lib/Helper test
  test.depends = lib/Jpeg lib/Helper
}

