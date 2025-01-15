//
// Created by mazen_2ax2a8u on 15/01/2025.
//

#ifndef BAL_H
#define BAL_H

#include <pthread.h>

typedef struct s_bal_t
{
    int data;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    unsigned int full;
} *bal_t;

/**
 * Create a new bounded buffer
 * @return a new bounded buffer
 */
bal_t bal_create();

/**
 * Destroy a bounded buffer
 * @param bal the bounded buffer to destroy
 */
void bal_destroy(bal_t bal);

/**
 * Put a new data in the bounded buffer
 * @param bal the bounded buffer
 * @param data the data to put
 */
void bal_put(bal_t bal, int data);

/**
 * Get a data from the bounded buffer
 * @param bal the bounded buffer
 * @return the data
 */
int bal_get(bal_t bal);


#endif //BAL_H
