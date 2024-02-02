function blkStruct = slblocks

    % This function specifies that the library should appear in the
    % Library Browser and be cached in the browser repository

    Browser.Library = 'WBC_lib_main';
    % the name of the library

    Browser.Name = 'WholeBodyControllers';
    % the library name that appears in the Library Browser

    blkStruct.Browser = Browser;
end
