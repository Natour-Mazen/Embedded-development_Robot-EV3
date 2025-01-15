//
// Created by mazen on 15/01/2025.
//

#include <stdlib.h>
#include <pthread.h>
#include "bal.h"

/**
 * Create a new bounded buffer
 * @return a new bounded buffer
 */
bal_t bal_create()
{
    bal_t bal = (bal_t)malloc(sizeof(struct s_bal_t));
    bal->data = 0;
    pthread_mutex_init(&bal->mutex, NULL);
    pthread_cond_init(&bal->cond, NULL);
    bal->full = 0;
    return bal;
}

/**
 * Destroy a bounded buffer
 * @param bal the bounded buffer to destroy
 */
void bal_destroy(bal_t bal)
{
    pthread_mutex_destroy(&bal->mutex);
    pthread_cond_destroy(&bal->cond);
    free(bal);
}

/**
 * Put a new data in the bounded buffer
 * @param bal the bounded buffer
 * @param data the data to put
 */
void bal_put(bal_t bal, int data)
{
    pthread_mutex_lock(&bal->mutex);
    while (bal->full)
    {
        pthread_cond_wait(&bal->cond, &bal->mutex);
    }
    bal->data = data;
    bal->full = 1;
    pthread_cond_signal(&bal->cond);
    pthread_mutex_unlock(&bal->mutex);
}

/**
 * Get a data from the bounded buffer
 * @param bal the bounded buffer
 * @return the data
 */
int bal_get(bal_t bal)
{
    pthread_mutex_lock(&bal->mutex);
    while (!bal->full)
    {
        pthread_cond_wait(&bal->cond, &bal->mutex);
    }
    int data = bal->data;
    bal->full = 0;
    pthread_cond_signal(&bal->cond);
    pthread_mutex_unlock(&bal->mutex);
    return data;
}