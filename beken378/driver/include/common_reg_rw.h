#ifndef __COMMON_REG_RW_H__
#define __COMMON_REG_RW_H__

#if CFG_USE_PATCH_FOR_QSPI_REG_WRITE
#define REG_WRITE_PROTECT(addr, value)    do{                         \
                                                GLOBAL_INT_DECLARATION(); \
                                                if((addr & 0x00FF8300) == 0x00A00000) {\
                                                    GLOBAL_INT_DISABLE();\
                                                    UINT32 qsio_reg = (addr & 0xff) + 0x00A02000;\
                                                    UINT32 qsio_val = REG_READ(qsio_reg);  \
                                                    REG_WRITE(addr, value);\
                                                    REG_WRITE(qsio_reg, qsio_val);\
                                                    GLOBAL_INT_RESTORE();\
                                                } else {\
                                                    REG_WRITE(addr, value);\
                                                }\
                                            }while(0)

#define REG_WRITE_QSPI_EN(addr, value)    do{                         \
                                                GLOBAL_INT_DECLARATION(); \
                                                if((addr & 0x00FF83FF) == 0x00A00060) {\
                                                    GLOBAL_INT_DISABLE();\
                                                    UINT32 qsio_reg = 0x00A02060;\
                                                    UINT32 qsio_val = REG_READ(qsio_reg);  \
                                                    REG_WRITE(addr, value);\
                                                    REG_WRITE(qsio_reg, qsio_val);\
                                                    qsio_reg = 0x00A02008;  \
                                                    qsio_val = REG_READ(qsio_reg);  \
                                                    qsio_val &= ~(1 << 0);  \
                                                    REG_WRITE(qsio_reg, qsio_val);\
                                                    qsio_val = REG_READ(qsio_reg);  \
                                                    qsio_val |= (1 << 0);  \
                                                    REG_WRITE(qsio_reg, qsio_val);\
                                                    GLOBAL_INT_RESTORE();\
                                                } else {\
                                                    REG_WRITE(addr, value);\
                                                }\
                                            }while(0)

#define REG_WRITE_QSPI_RST(addr, value)    do{                         \
                                                GLOBAL_INT_DECLARATION(); \
                                                if((addr & 0x00FF83FF) == 0x00A00008) {\
                                                    GLOBAL_INT_DISABLE();\
                                                    UINT32 qsio_reg = 0x00A02060;\
                                                    UINT32 qsio_val = REG_READ(qsio_reg);  \
                                                    REG_WRITE(addr, value);\
                                                    if(qsio_val & 0x1) { \
                                                        qsio_val &= ~(1 << 0);  \
                                                        REG_WRITE(qsio_reg, qsio_val);\
                                                        qsio_val |= (1 << 0);  \
                                                        REG_WRITE(qsio_reg, qsio_val);\
                                                        qsio_reg = 0x00A02008;  \
                                                        qsio_val = REG_READ(qsio_reg);  \
                                                        qsio_val &= ~(1 << 0);  \
                                                        REG_WRITE(qsio_reg, qsio_val);\
                                                        qsio_val = REG_READ(qsio_reg);  \
                                                        qsio_val |= (1 << 0);  \
                                                        REG_WRITE(qsio_reg, qsio_val);\
                                                    } \
                                                    GLOBAL_INT_RESTORE();\
                                                } else {\
                                                    REG_WRITE(addr, value);\
                                                }\
                                            }while(0)

#else
#define REG_WRITE_PROTECT(addr, value)    REG_WRITE(addr, value)
#define REG_WRITE_QSPI_EN(addr, value)    REG_WRITE(addr, value)
#define REG_WRITE_QSPI_RST(addr, value)   REG_WRITE(addr, value)
#endif

#endif // __COMMON_REG_RW_H__