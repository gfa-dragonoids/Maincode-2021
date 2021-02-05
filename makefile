BASEDIR=$(CURDIR)
OUTPUTDIR=$(BASEDIR)/docs
PACKAGE=org.firstinspires.ftc.teamcode

html:
    javadoc "$(PACKAGE)" -d "$(OUTPUTDIR)" -encoding UTF-8

.PHONY: html
