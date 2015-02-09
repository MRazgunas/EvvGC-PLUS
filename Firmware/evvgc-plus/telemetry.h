/*
    EvvGC-PLUS - Copyright (C) 2013-2014 Sarunas Vaitekonis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

typedef struct tagDataHdr {
  uint8_t cmd_id;
  uint8_t size;
  const uint8_t *data;
  uint32_t crc;
} __attribute__((packed)) DataHdr, *PDataHdr;

/* Console input/output. */
extern BaseChannel *g_chnp;

#ifdef __cplusplus
extern "C" {
#endif
void telemetryReadSerialData(void);
#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H_ */
