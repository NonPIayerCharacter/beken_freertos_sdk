// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __BEKENDRIVERMFIAUTH_H__
#define __BEKENDRIVERMFIAUTH_H__

#pragma once
#include "include.h"

/** @addtogroup BK_PLATFORM
* @{
*/


/** @defgroup BK_MFIAUTH _BK_ MFiAuth
  * @brief  Provide APIs for Apple Authentication Coprocessor operations
  * @{
  */

/******************************************************
 *                   Macros
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/



/******************************************************
 *                 Function Declarations
 ******************************************************/

/** @brief     PlatformMFiAuthInitialize
 *
 *  @abstract Performs any platform-specific initialization needed.
 *            Example: Bring up I2C interface for communication with
 *            the Apple Authentication Coprocessor.
 *
 *  @param   i2c            :  bk_i2c_t context
 *
 *  @return    kNoErr        : on success.
 *  @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkMFiAuthInitialize( bk_i2c_t i2c );


/** @brief    PlatformMFiAuthFinalize
 *
 *  @abstract Performs any platform-specific cleanup needed.
 *            Example: Bringing down the I2C interface for communication with
 *            the Apple Authentication Coprocessor.
 *
 *  @param    i2c            :  bk_i2c_t context
 *
 *  @return   none
 */
void BkMFiAuthFinalize( void );



/** @brief    PlatformMFiAuthCreateSignature
 *
 *  @abstract Create an RSA signature from the specified SHA-1 digest
 *            using the Apple Authentication Coprocessor.
 *
 *  @param    inDigestPtr     :    Pointer to 20-byte SHA-1 digest.
 *  @param    inDigestLen     :    Number of bytes in the digest. Must be 20.
 *  @param    outSignaturePtr :    Receives malloc()'d ptr to RSA signature. Caller must free() on success.
 *  @param    outSignatureLen :    Receives number of bytes in RSA signature.
 *
 *  @return    kNoErr         :    on success.
 *  @return    kGeneralErr    :    if an error occurred with any step
 */
OSStatus BkMFiAuthCreateSignature( const  void      *inDigestPtr,
                                   size_t     inDigestLen,
                                   uint8_t  **outSignaturePtr,
                                   size_t    *outSignatureLen );



/** @brief    PlatformMFiAuthCopyCertificate
 *
 *  @abstract Copy the certificate from the Apple Authentication Coprocessor.
 *
 *  @param    outCertificatePtr:   Receives malloc()'d ptr to a DER-encoded PKCS#7 message containing the certificate.
                                    Caller must free() on success.
 *  @param    outCertificateLen:   Number of bytes in the DER-encoded certificate.
 *
 *  @return    kNoErr         :    on success.
 *  @return    kGeneralErr    :    if an error occurred with any step
 */
OSStatus BkMFiAuthCopyCertificate( uint8_t **outCertificatePtr, size_t *outCertificateLen );

/** @} */
/** @} */
#endif // __BEKENDRIVERMFIAUTH_H__


