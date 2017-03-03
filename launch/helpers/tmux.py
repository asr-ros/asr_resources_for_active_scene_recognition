#Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Stroh Daniel, Trautmann Jeremias
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other #materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific #prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED #WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, #INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR #PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) #ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import subprocess
import time
from string_util import *
import libtmux

server = libtmux.Server()
session = server.sessions[0]
curWindow = session.attached_window
curPane = session.attached_pane

# tmux api
# windows

def getPaneOut(pane):
  return "\n".join(pane.cmd("capturep", "-p").stdout)

def getTmuxCurOut():
  return getPaneOut(session.attached_pane)

def getTmuxOut(windowNameOrId, paneId):
  window = getWindow(windowNameOrId)
  if window is None:
    return
  paneId = paneId % len(window.panes)
  return getPaneOut(window.panes[paneId])

def getWindow(windowNameOrId):
  if type(windowNameOrId) is int:
    windowNameOrId = "@" + str(windowNameOrId)
  window = session.get_by_id(windowNameOrId)
  if window is not None:
    return window
  # otherwise we have to find the window
  # [(name, window)]
  windowTuples = map(lambda x : (x._info["window_name"], x), session.windows)
  filteredWindows = filter (lambda (name, window): name == windowNameOrId, windowTuples)
  if len(filteredWindows) > 0:
    return filteredWindows[0][1]

def selectWindow(windowNameOrId):
  window = getWindow(windowNameOrId)
  if window is not None:
    window.select_window()
  else:
    print("window not found")

def getNumberOfWindows():
  return len(session.windows)

def getValidWindowIds():
  windows = session.windows
  validWindowIds = map(lambda x: x._window_id, windows)
  return validWindowIds

def restartWindow(i):
  selectWindow(str(i))
  nPanes = getNumberOfPanes()
  for i in range(0, nPanes):
    selectPane(i)
    restartCurrentPane()

def terminateWindow(i):
  selectWindow(str(i))
  nPanes = getNumberOfPanes()
  for i in range(0, nPanes):
    selectPane(i)
    terminatePane()

def restartAllWindows():
  for i in getValidWindowIds():
    restartWindow(i)

def terminateAllWindows():
  for i in getValidWindowIds():
    terminateWindow(i)

# panes
def getNumberOfPanes():
  return len(session.attached_window.panes)

def selectPane(paneId):
  nPanes = getNumberOfPanes()
  paneId = paneId % nPanes
  session.attached_window.panes[paneId].select_pane()

def terminatePane():
  session.attached_pane.cmd("send-keys", "C-C")

def restartCurrentPane():
  terminatePane()
  # wait for application to finish
  while True:
    lastLine = getLastLinesFrom(getTmuxCurOut(), -1)
    lastSymbol = lastLine[-1] if len(lastLine) > 0 else ""
    if "$" == lastSymbol:
      break
    terminatePane()
    # to terminate python process, we first remove all input characters before and after and then use Ctrl + d
    #session.attached_pane.cmd("send-keys", "C-k")
    #session.attached_pane.cmd("send-keys", "C-u")
    #session.attached_pane.cmd("send-keys", "C-d")
    time.sleep(1)
  # restart last command
  session.attached_pane.cmd("send-keys", "Up")
  session.attached_pane.enter()

def terminateCurrentPane():
  terminatePane()
  # wait for application to finish
  while True:
    lastLine = getLastLinesFrom(getTmuxCurOut(), -1)
    lastSymbol = lastLine[-1] if len(lastLine) > 0 else ""
    if "$" == lastSymbol:
      break
    terminatePane()
    time.sleep(1)

def restartPanes(windowAndPanes):
  oldPane = session.attached_pane
  if type(windowAndPanes) is not list:
    windowAndPanes = [windowAndPanes]
  for (windowNameOrId, paneId) in windowAndPanes:
    selectWindow(windowNameOrId)
    selectPane(paneId)
    restartCurrentPane()
  oldPane.window.select_window()
  oldPane.select_pane()

def terminatePanes(windowAndPanes):
  oldPane = session.attached_pane
  if type(windowAndPanes) is not list:
    windowAndPanes = [windowAndPanes]
  for (windowNameOrId, paneId) in windowAndPanes:
    selectWindow(windowNameOrId)
    selectPane(paneId)
    terminateCurrentPane()
  oldPane.window.select_window()
  oldPane.select_pane()

def selectWindowAndPane(windowNameOrId, paneId):
  selectWindow(windowNameOrId)
  selectPane(paneId)

def tmuxKill():
  subprocess.call("tmux kill-server", shell=True)
