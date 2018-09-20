function varargout = torqueBalancingGUI(varargin)
% TORQUEBALANCINGGUI MATLAB code for torqueBalancingGUI.fig
%      TORQUEBALANCINGGUI, by itself, creates a new TORQUEBALANCINGGUI or raises the existing
%      singleton*.
%
%      H = TORQUEBALANCINGGUI returns the handle to a new TORQUEBALANCINGGUI or the handle to
%      the existing singleton*.
%
%      TORQUEBALANCINGGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TORQUEBALANCINGGUI.M with the given input arguments.
%
%      TORQUEBALANCINGGUI('Property','Value',...) creates a new TORQUEBALANCINGGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before torqueBalancingGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to torqueBalancingGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help torqueBalancingGUI

% Last Modified by GUIDE v2.5 19-Sep-2018 16:37:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State     = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @torqueBalancingGUI_OpeningFcn, ...
                       'gui_OutputFcn',  @torqueBalancingGUI_OutputFcn, ...
                       'gui_LayoutFcn',  [] , ...
                       'gui_Callback',   []);
               
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before torqueBalancingGUI is made visible.
function torqueBalancingGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to torqueBalancingGUI (see VARARGIN)

% Choose default command line output for torqueBalancingGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes torqueBalancingGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% synchronization of the push button status with Simulink GUI
assignin('base','sl_synch_handles',handles)


% --- Outputs from this function are returned to the command line.
function varargout = torqueBalancingGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles) %#ok<*INUSL,*DEFNU>
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

mystring = get(hObject,'String');
status   = get_param(bdroot,'simulationstatus');
   
   
if strcmp(mystring,'Start Simulation')
    
    % Check the status of the simulation and start it if it's stopped
    if strcmp(status,'stopped')
        set_param(bdroot,'simulationcommand','start')
    end
    
    % Update the string on the pushbutton
    set(handles.pushbutton1,'String','Stop Simulation')
    
elseif strcmp(mystring,'Stop Simulation')
    
    % Check the status of the simulation and stop it if it's running
    if strcmp(status,'running')
        set_param(bdroot, 'SimulationCommand', 'Stop')
    end
    
    % Update the string on the pushbutton
    set(handles.pushbutton1,'String','Start Simulation')
    
else
    warning('Unrecognized string for pushbutton1') %#ok<WNTAG>
end

% Assign handles and the startstop object to the base workspace
assignin('base','sl_synch_handles',handles)
assignin('base','hObject',handles.pushbutton1)


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2
assignin('base','sl_synch_handles',handles)
 
