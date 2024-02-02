function varargout = simulinkStaticGUI(varargin)

    % SIMULINKSTATICGUI MATLAB code for simulinkStaticGUI.fig
    %
    %      SIMULINKSTATICGUI, by itself, creates a new SIMULINKSTATICGUI or raises the existing
    %      singleton*.
    %
    %      H = SIMULINKSTATICGUI returns the handle to a new SIMULINKSTATICGUI or the handle to
    %      the existing singleton*.
    %
    %      SIMULINKSTATICGUI('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in SIMULINKSTATICGUI.M with the given input arguments.
    %
    %      SIMULINKSTATICGUI('Property','Value',...) creates a new SIMULINKSTATICGUI or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before simulinkStaticGUI_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to simulinkStaticGUI_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help simulinkStaticGUI

    % Last Modified by Gabriele Nava, 28-Feb-2019

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
        'gui_Singleton',  gui_Singleton, ...
        'gui_OpeningFcn', @simulinkStaticGUI_OpeningFcn, ...
        'gui_OutputFcn',  @simulinkStaticGUI_OutputFcn, ...
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
end

% --- Executes just before simulinkStaticGUI is made visible.
function simulinkStaticGUI_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<INUSL>

    % This function has no output args, see OutputFcn.
    %
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to simulinkStaticGUI (see VARARGIN)

    % Choose default command line output for simulinkStaticGUI
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes simulinkStaticGUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);

    % synchronization of the push button status with Simulink GUI
    assignin('base','sl_synch_handles',handles)
end

% --- Outputs from this function are returned to the command line.
function varargout = simulinkStaticGUI_OutputFcn(hObject, eventdata, handles)  %#ok<INUSL>

    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

% --- Executes on button press in compileButton.
function compileButton_Callback(hObject, eventdata, handles)  %#ok<INUSL>

    % COMPILE MODEL BUTTON
    %
    % hObject    handle to compileButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % compile the model
    compileModel;

    if ~exist('errorMessages','var')

        % enable the start/stop button
        set(handles.startButton,'Enable','on')
        set(handles.startButton,'Backgroundcolor','g');
    else
        error(errorMessages.message)
    end
end

% --- Executes on button press in startButton.
function startButton_Callback(hObject, eventdata, handles) %#ok<INUSL>

    % START MODEL BUTTON
    %
    % hObject    handle to startButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    status = get_param(bdroot,'simulationstatus');

    if strcmp(status,'stopped')

        set_param(bdroot,'simulationcommand','start')

        % Deactivate all buttons, including the start button itself
        set(handles.startButton,'Backgroundcolor',[0.8,0.8,0.8]);
        set(handles.startButton,'Enable','off')
        set(handles.compileButton,'Backgroundcolor',[0.8,0.8,0.8]);
        set(handles.compileButton,'Enable','off')
        set(handles.exitButton,'Backgroundcolor',[0.8,0.8,0.8]);
        set(handles.exitButton,'Enable','off')

    else
        error('Model is already running')
    end

    % Assign handles and the startstop object to the base workspace
    assignin('base','sl_synch_handles',handles)
end

% --- Executes on button press in stopModelButton.
function stopButton_Callback(hObject, eventdata, handles) %#ok<INUSL>

    % STOP MODEL BUTTON
    %
    % hObject    handle to stopModelButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    status = get_param(bdroot,'simulationstatus');

    % Check the status of the simulation and stop it if it's running
    if strcmp(status,'running')

        % Disable the start button unless the user checked the proper
        % checkbox for avoiding to disable it
        if get(handles.checkbox_recompile, 'Value')

            set(handles.startButton,'Backgroundcolor',[0.8,0.8,0.8]);
            set(handles.startButton,'Enable','off')
        end

        % Enable compile and exit buttons
        set(handles.compileButton,'Backgroundcolor',[1.0,0.6,0.0]);
        set(handles.compileButton,'Enable','on')
        set(handles.exitButton,'Backgroundcolor',[0.0,1.0,1.0]);
        set(handles.exitButton,'Enable','on')

        set_param(bdroot, 'SimulationCommand', 'Stop')
    else
        error('Model is not running')
    end

    % Assign handles and the startstop object to the base workspace
    assignin('base','sl_synch_handles',handles)
end

% --- Executes on button press in exitButton.
function exitButton_Callback(hObject, eventdata, handles) %#ok<INUSL>

    % EXIT MODEL BUTTON
    %
    % hObject    handle to exitButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    closeModel;

    % Assign handles and the startstop object to the base workspace
    assignin('base','sl_synch_handles',handles)
end

% --- Executes on button press in checkbox_recompile.
function checkbox_recompile_Callback(hObject, eventdata, handles) %#ok<INUSD>

    % hObject    handle to checkbox_recompile (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: get(hObject,'Value') returns toggle state of checkbox_recompile
end
