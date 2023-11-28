function slBusOut = PointField(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.Name_SL_Info.ReceivedLength = uint32(strlength(msgIn.Name));
    currlen  = min(slBusOut.Name_SL_Info.ReceivedLength, length(slBusOut.Name));
    slBusOut.Name_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.Name(1:currlen) = uint8(char(msgIn.Name(1:currlen))).';
    slBusOut.Offset = uint32(msgIn.Offset);
    slBusOut.Datatype = uint8(msgIn.Datatype);
    slBusOut.Count = uint32(msgIn.Count);
end
