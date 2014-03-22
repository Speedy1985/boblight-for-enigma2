/*
 * boblight
 * Copyright (C) Bob  2009 
 * 
 * boblight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * boblight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "threading.h"

CThread::CThread()
{  
  thread_1 = 0;
  thread_2 = 0;
  m_running = false;
}

CThread::~CThread()
{
  StopThreads();
}

void CThread::startThreads()
{
  m_stop = false;
  m_running = true;
  pthread_create(&thread_1, 0, Thread_1, reinterpret_cast<void*>(this));
  //pthread_create(&thread_2, 0, Thread_2, reinterpret_cast<void*>(this));
}


void* CThread::Thread_1(void* args)
{
  CThread* thread_1 = reinterpret_cast<CThread*>(args);
  thread_1->socketProcess();
  thread_1->m_running = false;
}

void* CThread::Thread_2(void* args)
{
  CThread* thread_2 = reinterpret_cast<CThread*>(args);
  thread_2->beamProcess();
  thread_2->m_running = false;
}

void CThread::socketProcess(){}
void CThread::beamProcess(){}

void CThread::StopThreads()
{
  AsyncStopThreads();
  JoinThreads();
}

void CThread::AsyncStopThreads()
{
  m_stop = true;
}

void CThread::JoinThreads()
{
  if (thread_1)
  {
    pthread_join(thread_1, 0);
    thread_1 = 0;
  }
  
  if (thread_2)
  {
    pthread_join(thread_2, 0);
    thread_2 = 0;
  }
}

bool CThread::IsRunning()
{
  return m_running;
}

