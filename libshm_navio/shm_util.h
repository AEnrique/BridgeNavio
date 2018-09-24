//----------------------------------------------------------------------------------------------------------------------
// GRVC AUTOPILOT
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#ifndef SHM_UTIL_H_
#define SHM_UTIL_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>


 ///  Create a global variable: static struct shm_str<yourType> shm_type;
template <typename Type> struct shm_str
{
	int _shmfd;
	int _shared_seg_size;
	const char *_channel_shm;
	const char *_channel_sem;
	Type *_shared_msg; // the shared segment, and head of the messages list
	sem_t *_semfd;
};

/// Inside main function: create and initialize the shared memory: create_shm<yourType>(&shm_type,"direction_in_memory","direction_of_the semaphore");
template<typename Type>
inline void create_shm(shm_str<Type> *shm,const char *channel_shm,const char *channel_sem)
{

	shm->_shared_seg_size = (1 * sizeof(Type));   // want shared segment capable of storing 1 message
	shm->_channel_shm = channel_shm;
	shm->_channel_sem = channel_sem;
	shm->_shmfd = shm_open(channel_shm, O_CREAT | O_RDWR, S_IRWXU | S_IRWXG);
	if (shm->_shmfd < 0)
	{
		perror("Error: In shm_open()");
		exit(1);
	}

	//fprintf(stderr, "Created shared memory object %s\n", channel);
	// adjusting mapped file size (make room for the whole segment to map)      --  ftruncate()
	ftruncate(shm->_shmfd, shm->_shared_seg_size);
	// requesting the shared segment    --  mmap()
	shm->_shared_msg = (Type*)mmap(NULL, shm->_shared_seg_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm->_shmfd, 0);
	if (shm->_shared_msg == NULL)
	{
		perror("In mmap()");
		exit(1);
	}
	//fprintf(stderr, "Shared memory segment allocated correctly (%d bytes).\n", shm->_shared_seg_size);
	shm->_semfd=sem_open(channel_sem, O_CREAT, S_IRUSR | S_IWUSR,1);

}


/// Use get_shm or set_shm to read or write from the shared memory. Create a variable <yourType> msg to do this;
template<typename Type>
inline int set_shm(shm_str<Type> *shm, Type msg)
{
	sem_wait(shm->_semfd);
	memcpy(shm->_shared_msg,&msg,sizeof(Type));
	sem_post(shm->_semfd);
	return sizeof(msg);
}


template<typename Type>
inline Type* get_shm(shm_str<Type> *shm)
{
	Type *msg;
	sem_wait(shm->_semfd);
	msg=shm->_shared_msg;
	sem_post(shm->_semfd);
	return msg;

}


/// Close and eliminate the shared memory. Use close_shm(shm_type);
template<typename Type>
inline int close_shm(shm_str<Type> *shm)
{
	if (shm_unlink(shm->_channel_shm) < 0 )
	{
		perror("shm_unlink");//Semaphore Close: Close a named semaphore
		return -1;
	}
	if (sem_close(shm->_semfd) < 0 )
	{
		perror("sem_close");//Semaphore unlink: Remove a named semaphore  from the system.
		return -1;
	}
	if (sem_unlink(shm->_channel_sem) < 0 )
	{
		perror("sem_unlink");
		return -1;
	}
	return 0;
}


#endif /* SHM_UTIL_H_ */

