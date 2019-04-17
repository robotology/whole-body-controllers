function varargout = vectorDemux(vector,partitionTable)

    % VECTORDEMUX demuxes a vector according to a user-defined partition.
    %
    % FORMAT:  varargout = vectorDemux(vector,partitionTable)
    %
    % INPUTS:  - vector: [n * 1] the vector to demux. 
    %          - partitionTable: a vector of positive integers that specifies 
    %                            how to demux the input vector.
    %
    % OUTPUTS: - the input vector demuxed according to the partition table.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Genova, Nov 2018

    %% ------------Initialization----------------

    % verify the partition table is valid
    if sum(partitionTable) ~= length(vector)
        
        error('[vectorDemux]: invalid partition table.')
    end
    
    % add an extra 0 as first element of the partitionTable to simplify
    % the input vector demux procedure
    if size(partitionTable,1) == 1
        
        partitionTable = [0, partitionTable];
    else
        partitionTable = [0; partitionTable];
    end
    
    % partition the input vector
     sumPartitionTable = 0;
     
    for k = 2:length(partitionTable)
       
        % zeros, non integer and negative elements are not allowed
        if partitionTable(k) <= 0 || partitionTable(k) ~= round(partitionTable(k))
            
            error('[vectorDemux]: the partition table contains an invalid element.');
        end
       
        % cumulate the values of the partition table
        sumPartitionTable = sumPartitionTable + partitionTable(k-1);
        varargout{k-1}    = vector(1+sumPartitionTable:sumPartitionTable+partitionTable(k)); %#ok<AGROW>
    end
end        