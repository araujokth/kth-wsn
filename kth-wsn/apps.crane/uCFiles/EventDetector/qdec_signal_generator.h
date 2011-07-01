#ifndef __QDEC_SIGNAL_GENERATOR_H__
#define __QDEC_SIGNAL_GENERATOR_H__

void generate_qdec_signal(PORT_t * qPort,
                          uint8_t lineCount,
                          uint8_t freq,
                          bool dir);
                          
/* this #pragma directive will tell the compiler to compile/link the
   functions from the mylib.lib library */
#pragma library qdec_signal_generator.lib                          

#endif /* __QDEC_SIGNAL_GENERATOR_H__ */