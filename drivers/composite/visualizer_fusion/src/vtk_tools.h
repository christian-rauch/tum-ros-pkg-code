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

#ifndef VTK_TOOLS_HH
#define VTK_TOOLS_HH

// VTK includes
#include <vtkCellArray.h>
#include <vtkPointData.h>
#include <vtkLODActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataWriter.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkXRenderWindowInteractor.h>
#include <vtkDataSetMapper.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkObjectFactory.h>
#include <vtkCamera.h>
#include <vtkAssemblyPath.h>
#include <vtkRendererCollection.h>
#include <vtkMultiThreader.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkCriticalSection.h>
#include <vtkTextActor.h>
#include <vtkProperty2D.h>

#define KEY_PLUS  43
#define KEY_MINUS 45

////////////////////////////////////////////////////////////////////////////////
class vtkWGInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static vtkWGInteractorStyle *New ();

    void OnChar ();

    void
      OnTimer ()
    {
//      lock_timer_->Lock ();
//      ren_win_->Render ();
//      lock_timer_->Unlock ();
//      this->Superclass::OnTimer ();
    }

    vtkPNGWriter* writer_;
    vtkWindowToImageFilter *filter_;
    vtkRenderWindow *ren_win_;

    vtkCriticalSection *lock_timer_, *lock_char_;

    int pointsize_;
  protected:
    char fileName_[80];
    char camFN_[80];
};


////////////////////////////////////////////////////////////////////////////////
class vtkFPSCallback : public vtkCommand
{
  public:
    static vtkFPSCallback *New () { return new vtkFPSCallback;}
    void SetTextActor (vtkTextActor *txt);
    virtual void Execute (vtkObject *caller, unsigned long, void*);
  protected:
    vtkTextActor *TextActor;
    char TextBuff[128];
};

#endif
