ifndef vsl_client_mk
vsl_client_mk := 1
#
# fsm@robots.ox.ac.uk
#

include $(IUELOCALROOT)/vxl/vcl/client.mk
include $(IUELOCALROOT)/vxl/vbl/client.mk
include $(IUELOCALROOT)/vxl/vnl/client.mk
include $(IUELOCALROOT)/vxl/vil/client.mk

ald_libdeps += vsl:vcl,vil,vbl,vnl-algo

endif
