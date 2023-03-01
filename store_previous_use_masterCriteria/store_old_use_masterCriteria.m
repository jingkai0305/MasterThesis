function store_old_use_masterCriteria(this, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters)
    %             associationMatrix =
    %             TrackToTrackAssociation(this.lookupTable)
    %             association = zeros(this.N_OBJ_TRACKS);
    % Extrinsic declarations.
    coder.extrinsic('warning');

    % Get which objects are considered associated.
    masterCriteria = TrackToTrackAssociation(this, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);

    % Update asssociationPointer field.
    for i = 1 : this.N_OBJ_TRACKS
        % Check if there are any associationPointer to be set.
        if sum(masterCriteria(i,:)) == 0 
            this.lookupTable(i).associationPointer = uint8(0);
            continue;

        elseif sum(masterCriteria(i,:)) == 1
            % Find the association pointer
            for j = 1 : this.N_OBJ_TRACKS
                if masterCriteria(i,j) == 1
                    % Set the found associationPointer. Assign
                    % the row number to be associated to the
                    % column.
                    this.lookupTable(j).associationPointer = uint8(i);
                    this.lookupTable(i).associationPointer = uint8(j);
                    this.lookupTable(j).ascSourceMemory = this.lookupTable(i).source;
                    this.lookupTable(j).ascIDMemory = this.lookupTable(i).ID;                            
                    this.lookupTable(i).ascSourceMemory = this.lookupTable(j).source;
                    this.lookupTable(i).ascIDMemory = this.lookupTable(j).ID;
                end
            end
        else
            warning('Wrong in masterCriteria for association.')
        end
    end
end