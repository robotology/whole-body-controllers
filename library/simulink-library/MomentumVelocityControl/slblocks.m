function blkStruct = slblocks
		% This function specifies that the library should appear
		% in the Library Browser
		% and be cached in the browser repository

		Browser.Library = 'momentumVelocityControl_lib'; % name of the library

		Browser.Name = 'MomentumVelocityControl'; % library name that appears in the Library Browser
        
        Browser.IsFlat  =  0;
        
        blkStruct.Browser = Browser;
