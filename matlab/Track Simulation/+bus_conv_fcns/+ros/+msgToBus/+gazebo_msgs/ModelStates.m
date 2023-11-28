function slBusOut = ModelStates(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    maxlength = length(slBusOut.Name);
    recvdlength = length(msgIn.Name);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Name', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Name_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Name_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        recvlen = strlength(msgIn.Name(iter));
        maxlen = length(slBusOut.Name(iter).Data);
        curlen = min(recvlen, maxlen);
        if (max(recvlen) > maxlen) && ...
                isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
            diag = MSLDiagnostic([], ...
                                 message('ros:slros:busconvert:TruncatedArray', ...
                                         'Name', msgIn.MessageType, maxlen, max(recvdlength), maxlength, varargin{2}));
            reportAsWarning(diag);
        end
        slBusOut.Name(iter).Data_SL_Info.CurrentLength = uint32(curlen);
        slBusOut.Name(iter).Data_SL_Info.ReceivedLength = uint32(recvlen);
        slBusOut.Name(iter).Data(1:curlen) = uint8(char(msgIn.Name(iter)));
    end
    maxlength = length(slBusOut.Pose);
    recvdlength = length(msgIn.Pose);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Pose', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Pose_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Pose_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        slBusOut.Pose(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose(iter),slBusOut(1).Pose(iter),varargin{:});
    end
    maxlength = length(slBusOut.Twist);
    recvdlength = length(msgIn.Twist);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Twist', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Twist_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Twist_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        slBusOut.Twist(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist(iter),slBusOut(1).Twist(iter),varargin{:});
    end
end
