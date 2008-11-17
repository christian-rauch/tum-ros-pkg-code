/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Radu Bogdan Rusu <rusu@cs.tum.edu> */

#include "vtk_tools.h"

vtkStandardNewMacro (vtkWGInteractorStyle);

void
  vtkWGInteractorStyle::OnChar ()
{
  if (pointsize_ < 1)
    pointsize_ = 1;

//  lock_char_->Lock ();
  // ---[ Check the rest of the key codes
  switch (this->Interactor->GetKeyCode ())
  {
    case KEY_PLUS:
    {
      vtkActorCollection *ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath *path;
      ac = this->CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
      {
        for (anActor->InitPathTraversal (); (path=anActor->GetNextPath ()); )
        {
          aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
          aPart->GetProperty ()->SetPointSize (aPart->GetProperty ()->GetPointSize () + 1);
        }
      }
      break;
    }
    case KEY_MINUS:
    {
      vtkActorCollection *ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath *path;
      ac = this->CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
      {
        for (anActor->InitPathTraversal (); (path=anActor->GetNextPath ()); )
        {
          aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
          aPart->GetProperty ()->SetPointSize (aPart->GetProperty ()->GetPointSize () - 1);
        }
      }
      break;
    }
  }
  this->Superclass::OnChar ();
//  lock_char_->Unlock ();
}

////////////////////////////////////////////////////////////////////////////////
void
  vtkFPSCallback::SetTextActor (vtkTextActor *txt)
{
  this->TextActor = txt;
}
void
  vtkFPSCallback::Execute (vtkObject *caller, unsigned long, void*)
{
  vtkRenderer *ren = reinterpret_cast<vtkRenderer *> (caller);

  long int nr_points = 0;
  vtkActorCollection *ac = ren->GetActors ();
  vtkActor *anActor, *aPart;
  vtkCollectionSimpleIterator ait;
  vtkAssemblyPath *path;
  for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
  {
    for (anActor->InitPathTraversal(); (path=anActor->GetNextPath ()); )
    {
      aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
      nr_points += aPart->GetMapper ()->GetInputAsDataSet ()->GetNumberOfPoints ();
    }
  }

  float fps = 1.0/ren->GetLastRenderTimeInSeconds ();
  snprintf (this->TextBuff, 127, "%.1f FPS, %ld points", fps, nr_points);
  this->TextActor->SetInput (this->TextBuff);
}
