function varargout = torqueBalancingGui(varargin)

    % TORQUEBALANCINGGUI MATLAB code for torqueBalancingGui.fig
    %
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
    %      applied to the GUI before torqueBalancingGui_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to torqueBalancingGui_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help torqueBalancingGui

    % Last Modified by GUIDE v2.5 13-Dec-2018 14:51:36

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @torqueBalancingGui_OpeningFcn, ...
                       'gui_OutputFcn',  @torqueBalancingGui_OutputFcn, ...
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

% --- Executes just before torqueBalancingGui is made visible.
function torqueBalancingGui_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<INUSL>

    % This function has no output args, see OutputFcn.
    %
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to torqueBalancingGui (see VARARGIN)

    % Choose default command line output for torqueBalancingGui
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes torqueBalancingGui wait for user response (see UIRESUME)
    % uiwait(handles.figure1);

    % synchronization of the push button status with Simulink GUI
    assignin('base','sl_synch_handles',handles)
end

% --- Outputs from this function are returned to the command line.
function varargout = torqueBalancingGui_OutputFcn(hObject, eventdata, handles)  %#ok<INUSL>

    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>

    % hObject    handle to pushbutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % PUSHBUTTON1 = COMPILE MODEL
    
    % compile the model
    compileModel;
    
    if ~exist('errorMessages','var')
    
        % enable the start/stop button
        set(handles.pushbutton5,'Enable','on')
        set(handles.pushbutton5,'Backgroundcolor','g');
    else
        error(errorMessages.message)
    end
end

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>

    % hObject    handle to pushbutton5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % PUSHBUTTON4 = CLOSE MODEL

    closeModel;

    % Assign handles and the startstop object to the base workspace
    assignin('base','sl_synch_handles',handles)
end

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>

    % hObject    handle to pushbutton4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % PUSHBUTTON5 = START/STOP MODEL

    mystring = get(hObject,'String');
    status   = get_param(bdroot,'simulationstatus');

    if strcmp(mystring,'Start Model')
    
        % Check the status of the simulation and start if it's stopped
        if strcmp(status,'stopped')
            set_param(bdroot,'simulationcommand','start')
        end
    
        % Update the string on the pushbutton
        set(handles.pushbutton5,'String','Stop Model')
        set(handles.pushbutton5,'Backgroundcolor','r');
    
    elseif strcmp(mystring,'Stop Model')
    
        % Check the status of the simulation and stop it if it's running
        if strcmp(status,'running')
            set_param(bdroot, 'SimulationCommand', 'Stop')
        end
    
        % Update the string on the pushbutton
        set(handles.pushbutton5,'String','Start Model')
        set(handles.pushbutton5,'Backgroundcolor',[0.8,0.8,0.8]);
        
        % disable the button (simulation terminated)
        set(handles.pushbutton5,'Enable','inactive');
    else
        warning('Unrecognized string for pushbutton4') %#ok<WNTAG>
    end

    % Assign handles and the startstop object to the base workspace
    assignin('base','sl_synch_handles',handles)
end
