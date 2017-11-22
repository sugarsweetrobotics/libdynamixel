set(hdrs DynamixelV1.h DynamixelV2.h Exception.h)


set(install_hdrs ${hdrs})

install(FILES ${install_hdrs} DESTINATION ${INC_INSTALL_DIR}
    COMPONENT library)

