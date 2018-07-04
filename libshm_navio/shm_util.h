/*
 * shm_util.h
 *
 *  Created on: 1/12/2017
 *      Author: GRVC
 *
 *      1. Create a global variable: static struct shm_str<yourType> shm_type;
 *      2. Inside main function: create and initialize the shared memory: create_shm<yourType>(&shm_type,"direction_in_memory","direction_of_the semaphore");
 *      3. Use get_shm or set_shm to read or write from the shared memory. Create a variable <yourType> msg to do this;
 *      4. Close and eliminate the shared memory. Use close_shm(shm_type);
 *
 */

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


template <typename Type> struct shm_str
{
	int _shmfd;
	int _shared_seg_size;
	const char *_channel_shm;
	const char *_channel_sem;
	Type *_shared_msg; // the shared segment, and head of the messages list
	sem_t *_semfd;
};

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename Type>
inline int set_shm(shm_str<Type> *shm, Type msg)
{
	sem_wait(shm->_semfd);
	memcpy(shm->_shared_msg,&msg,sizeof(Type));
	sem_post(shm->_semfd);
	return sizeof(msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename Type>
inline Type* get_shm(shm_str<Type> *shm)
{
	Type *msg;
	sem_wait(shm->_semfd);
	msg=shm->_shared_msg;
	sem_post(shm->_semfd);
	return msg;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
