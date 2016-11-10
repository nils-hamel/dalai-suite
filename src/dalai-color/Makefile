
    include Version

#
#   make - Configuration
#

    CONFIG_NAME:=dalai-color
    CONFIG_CODE:=cpp
    CONFIG_TYPE:=suite

    MAKE_BINARY:=bin
    MAKE_DOCUME:=doc
    MAKE_LIBRAR:=lib
    MAKE_OBJECT:=obj
    MAKE_SOURCE:=src
    MAKE_BNPATH:=/media/dhlab-linux/bin/dalai-suite/bin
    MAKE_HDPATH:=/media/dhlab-linux/usr/include
    MAKE_CMCOPY:=cp
    MAKE_CMRMFL:=rm -f
    MAKE_CMRMRF:=rm -rf
    MAKE_CMMKDR:=mkdir -p
    MAKE_CMRMDR:=rmdir -p --ignore-fail-on-non-empty
    MAKE_CC_BLD:=gcc
    MAKE_CPPBLD:=g++
    MAKE_STABLD:=ar
    MAKE_DOCBLD:=doxygen

    BUILD_FLAGS:=-Wall -funsigned-char -O3 -I/usr/include/pcl-1.7 -I/usr/include/eigen3
    BUILD_LINKD:=-rdynamic -lboost_system -lboost_filesystem -lboost_thread -lboost_date_time -lboost_iostreams -lboost_serialization -lboost_chrono -lboost_atomic -lboost_regex -lqhull -lOpenNI -Wl,-Bstatic -lflann_cpp_s -Wl,-Bdynamic /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.2.so.6.2.0 -ljpeg -lpng -ltiff /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.2.so.6.2.0 -ljsoncpp -lexpat /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.2.so.6.2.0 -lgl2ps /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0 /usr/lib/libvtkWrappingTools-6.2.a /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.2.so.6.2.0 -lsqlite3 /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeOpenGL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.2.so.6.2.0 -lpcl_common -lpcl_kdtree -lpcl_octree -lpcl_search -lpcl_io -lpcl_sample_consensus -lpcl_filters -lpcl_features -lpcl_segmentation -lpcl_surface -lpcl_registration -lpcl_recognition -lpcl_keypoints -lpcl_visualization -lpcl_people -lpcl_outofcore -lpcl_tracking /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.2.so.6.2.0 -lxml2 -lpthread /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so -lsz -lm /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.2.so.6.2.0 -lnetcdf_c++ -lnetcdf /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1 /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1 /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1 /usr/lib/x86_64-linux-gnu/libvtkverdict-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0 -lpython2.7 /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0 -lGLU -lSM -lICE -lX11 -lXext -lXt /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0 -lfreetype -lGL /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.2.so.6.2.0 -ltheoraenc -ltheoradec -logg /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0 -ldl /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0 /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0 -lz -Wl,-rpath,/usr/lib/x86_64-linux-gnu/hdf5/serial/lib
    BUILD_SUBMD:=$(MAKE_LIBRAR)/libcommon

#
#   make - Modules
#

    MAKE_MODULE:=$(foreach LIBS,$(BUILD_SUBMD),$(if $(findstring /lib/,$(LIBS)),,$(LIBS)))

#
#   make - Auto-configuration
#

ifeq ($(CONFIG_TYPE),suite)
    MAKE_LIBSWAP:=../../
else
ifeq ($(CONFIG_TYPE),libcommon)
    MAKE_LIBSWAP:=../../
else
    MAKE_LIBSWAP:=
endif
endif
ifeq ($(CONFIG_TYPE),libstatic)
ifeq ($(STATIC),true)
    MAKE_SUFFIX:=.a
else
    MAKE_SUFFIX:=.so
endif
else
ifeq ($(CONFIG_TYPE),libcommon)
    MAKE_SUFFIX:=.a
else
    MAKE_SUFFIX:=
endif
endif
ifeq ($(CONFIG_CODE),c)
ifeq ($(CONFIG_TYPE),libstatic)
ifeq ($(STATIC),true)
    MAKE_LINKER:=$(MAKE_STABLD)
else
    MAKE_LINKER:=$(MAKE_CC_BLD)
    MAKE_OPTION:=$(MAKE_OPTION) -fpic
endif
else
ifeq ($(CONFIG_TYPE),libcommon)
    MAKE_LINKER:=$(MAKE_STABLD)
else
    MAKE_LINKER:=$(MAKE_CC_BLD)
endif
endif
    MAKE_HEADEX:=h
    MAKE_COMPIL:=$(MAKE_CC_BLD)
    MAKE_OPTION:=$(MAKE_OPTION) -std=gnu99
else
ifeq ($(CONFIG_CODE),cpp)
ifeq ($(CONFIG_TYPE),libstatic)
ifeq ($(STATIC),true)
    MAKE_LINKER:=$(MAKE_STABLD)
else
    MAKE_LINKER:=$(MAKE_CPPBLD)
    MAKE_OPTION:=$(MAKE_OPTION) -fpic
endif
else
ifeq ($(CONFIG_TYPE),libcommon)
    MAKE_LINKER:=$(MAKE_STABLD)
else
    MAKE_LINKER:=$(MAKE_CPPBLD)
endif
endif
    MAKE_HEADEX:=hpp
    MAKE_COMPIL:=$(MAKE_CPPBLD)
    MAKE_OPTION:=$(MAKE_OPTION) -std=c++11
endif
endif
ifneq ($(OPENMP),false)
    MAKE_OPTION:=$(MAKE_OPTION) -fopenmp -D __OPENMP__
    BUILD_LINKD:=$(BUILD_LINKD) -lgomp
endif
    MAKE_OPTION:=$(MAKE_OPTION) $(BUILD_FLAGS) $(addprefix -I./$(MAKE_LIBSWAP),$(addsuffix /src,$(BUILD_SUBMD)))
    MAKE_BUILDD:=$(MAKE_BUILDD) $(addprefix -l,$(subst lib,,$(notdir $(BUILD_SUBMD)))) $(BUILD_LINKD) $(addprefix -L./$(MAKE_LIBSWAP),$(addsuffix /bin,$(BUILD_SUBMD)))

ifeq ($(CONFIG_TYPE),libstatic)
    MAKE_VERSIO:=$(VER_MAJ).$(VER_MIN)$(if $(filter $(VER_REV),0),,.$(VER_REV))
    MAKE_HDPATH:=$(MAKE_HDPATH)/$(subst lib,,$(CONFIG_NAME))/$(MAKE_VERSIO)
endif

#
#   make - Enumeration
#

    MAKE_SRCFILE:=$(wildcard $(MAKE_SOURCE)/*.$(CONFIG_CODE))
    MAKE_OBJFILE:=$(addprefix $(MAKE_OBJECT)/,$(addsuffix .o,$(notdir $(basename $(MAKE_SRCFILE)))))

#
#   make - Targets
#

ifeq ($(CONFIG_TYPE),suite)
    all:make-directories $(CONFIG_NAME)
    build:all
    modules:
    clean:make-clean
    clean-all:clean
    clean-modules:
else
ifeq ($(CONFIG_TYPE),libcommon)
    all:make-directories $(CONFIG_NAME)
    build:all
    modules:
    clean:make-clean
    clean-all:clean
    clean-modules:
else
    all:make-directories make-modules $(CONFIG_NAME)
    build:make-directories $(CONFIG_NAME)
    modules:make-modules
    clean:make-clean
    clean-all:make-clean make-clean-modules
    clean-modules:make-clean-modules
endif
endif
    documentation:make-directories make-documentation
    clean-documentation:make-clean-documentation
    install:make-install
    uninstall:make-uninstall

#
#   make - Directives
#

    $(CONFIG_NAME):$(MAKE_OBJFILE)
ifeq ($(CONFIG_TYPE),libstatic)
ifneq ($(STATIC),false)
	$(MAKE_LINKER) rcs $(MAKE_BINARY)/$(CONFIG_NAME)$(MAKE_SUFFIX) $^
else
	$(MAKE_LINKER) -shared -Wl,-soname,$(CONFIG_NAME)$(MAKE_SUFFIX).$(MAKE_VERSIO) -o $(MAKE_BINARY)/$(CONFIG_NAME)$(MAKE_SUFFIX) $^
endif
else
ifeq ($(CONFIG_TYPE),libcommon)
	$(MAKE_LINKER) rcs $(MAKE_BINARY)/$(CONFIG_NAME)$(MAKE_SUFFIX) $^
else
	$(MAKE_LINKER) -o $(MAKE_BINARY)/$(CONFIG_NAME)$(MAKE_SUFFIX) $^ $(MAKE_BUILDD)
endif
endif

    $(MAKE_OBJECT)/%.o:$(MAKE_SOURCE)/%.$(CONFIG_CODE)
	$(MAKE_COMPIL) -c -o $@ $< $(MAKE_OPTION)

    make-modules:
	@$(foreach LIB, $(MAKE_MODULE), $(MAKE) -C $(LIB) all OPENMP=$(OPENMP) && ) true

    make-documentation:make-directories
	$(MAKE_DOCBLD)

#
#   make - Cleaning
#

    make-clean:
	$(MAKE_CMRMFL) $(MAKE_BINARY)/* $(MAKE_OBJECT)/*.o

    make-clean-modules:
	@$(foreach LIBS, $(MAKE_MODULE), $(MAKE) -C $(LIBS) clean-all && ) true

    make-clean-documentation:
	$(MAKE_CMRMRF) $(MAKE_DOCUME)/html

#
#   make - Implementation
#

    make-install:
ifeq ($(CONFIG_TYPE),libstatic)
	$(MAKE_CMCOPY) $(MAKE_BINARY)/$(CONFIG_NAME)$(MAKE_SUFFIX) $(MAKE_BNPATH)/$(CONFIG_NAME)$(MAKE_SUFFIX).$(MAKE_VERSIO)
	$(MAKE_CMMKDR) $(MAKE_HDPATH)
	$(MAKE_CMCOPY) $(addprefix $(MAKE_SOURCE)/,$(notdir $(wildcard $(MAKE_SOURCE)/*.$(MAKE_HEADEX)))) $(MAKE_HDPATH)
else
	$(MAKE_CMCOPY) $(MAKE_BINARY)/$(CONFIG_NAME)$(MAKE_SUFFIX) $(MAKE_BNPATH)/$(CONFIG_NAME)$(MAKE_SUFFIX)
endif

    make-uninstall:
ifeq ($(CONFIG_TYPE),libstatic)
	$(MAKE_CMRMFL) $(addprefix $(MAKE_HDPATH)/,$(notdir $(wildcard $(MAKE_SOURCE)/*.$(MAKE_HEADEX))))
	$(MAKE_CMRMDR) $(MAKE_HDPATH)
	$(MAKE_CMRMFL) $(MAKE_BNPATH)/$(CONFIG_NAME)$(MAKE_SUFFIX).$(MAKE_VERSIO)
else
	$(MAKE_CMRMFL) $(MAKE_BNPATH)/$(CONFIG_NAME)$(MAKE_SUFFIX)
endif

#
#   make - Directories
#

    make-directories:
	$(MAKE_CMMKDR) $(MAKE_BINARY) $(MAKE_DOCUME) $(MAKE_OBJECT)

