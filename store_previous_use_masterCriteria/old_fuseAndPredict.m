function old_fuseAndPredict(this, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, MAX_COASTING, fusionCycleTimeMs, dacuTime, parameters)
            
            coder.extrinsic('warning')
            % Fetch populated sensorTracks
            allOldSystemTrack = getSystemOutput(this);
            oldLookupTable = this.lookupTable;
            % Update systemTracks with all currently seen sensorTracks
            for i = 1 : this.N_OBJ_TRACKS
                % Check if table idx is already fused into system.
                if this.lookupTable(i).fused == true
                    continue;
                    % Check if table idx is valid or not.
                elseif this.lookupTable(i).source == uint8(0)
                    continue
                end                
                % Index
                trackIdx = uint8(i); %CurrentTrackIndex
                trackAscIdx = this.lookupTable(trackIdx).associationPointer;
                % Id
                trackId = this.lookupTable(trackIdx).ID;
                trackSource = this.lookupTable(trackIdx).source;
                % System Index
                % Note that it will not return Idx for coasted object
                oldTrackSystemIdx = this.isMemberOfSystemTrack(trackIdx, FLRdata, FLCdata, SRXdata);                                     
                latPos = this.lookupTable(trackIdx).closestSideLat;                
                longPos = this.lookupTable(trackIdx).closestSideLong;
                
                if trackAscIdx ~= uint8(0)
                    trackAscId = this.lookupTable(trackAscIdx).ID;
                    trackAscSource = this.lookupTable(trackAscIdx).source;
                    if trackAscSource == uint8(7)
                        oldTrackSystemAscIdx = this.lookupTable(trackAscIdx).signalNo;
                    else                       
                        oldTrackSystemAscIdx = this.isMemberOfSystemTrack(trackAscIdx, FLRdata, FLCdata, SRXdata);
                    end
                     latPosAsc = this.lookupTable(trackAscIdx).closestSideLat;
                     longPosAsc = this.lookupTable(trackAscIdx).closestSideLong;
                else
                    trackAscId = uint8(0);
                    trackAscSource = uint8(0);
                    oldTrackSystemAscIdx = uint8(0);
                    latPosAsc = single(0);
                    longPosAsc = single(0);
                end
                
                % Calculating sensors life time and system track age
                lifeTimeTrack = 0;
                lifeTimeAscTrack = 0;
                systemAge = 0;
                systemAscAge = 0;
                lifeTimeoldAscTrack = 0;
                lifeTimeoldAscAscTrack = 0;
        
                if trackSource ~= 0 && trackIdx ~= 0
                    lifeTimeTrack = double(BoschSensors.getObjectLifeTime(this, FLRdata.Signals, FLCdata.Signals, SRXdata, ...
                        trackSource, this.lookupTable(trackIdx).signalNo));
                end
                
                if trackAscSource ~= 0 && trackAscIdx ~= 0
                    lifeTimeAscTrack = double(BoschSensors.getObjectLifeTime(this, FLRdata.Signals, FLCdata.Signals, SRXdata, ...
                        trackAscSource, this.lookupTable(trackAscIdx).signalNo));
                end
                              
                if oldTrackSystemIdx ~= 0
                    systemAge = double(this.getSystemObjAge(oldTrackSystemIdx));   
                end
                
                if oldTrackSystemAscIdx ~= 0
                    systemAscAge = double(this.getSystemObjAge(oldTrackSystemAscIdx));
                end
                     
                if oldTrackSystemIdx ~= 0
                    objClassTrack = uint8(allOldSystemTrack(oldTrackSystemIdx).Estimate.objClass);
                    objClassSourceTrack = uint8(allOldSystemTrack(oldTrackSystemIdx).Estimate.objClassSource);
                else
                    objClassTrack = uint8(0);
                    objClassSourceTrack = uint8(0);
                end

                if oldTrackSystemAscIdx ~= 0
                    objClassAscTrack = uint8(allOldSystemTrack(oldTrackSystemAscIdx).Estimate.objClass);                      
                    objClassAscSourceTrack = uint8(allOldSystemTrack(oldTrackSystemAscIdx).Estimate.objClassSource);
                else
                    objClassAscTrack = uint8(0);
                    objClassAscSourceTrack = uint8(0);
                end

                %% Check for possible systemTracks to store the objects
                % Flags for easy call of fuseSystemTrack
                populateOldSystemTrack = false;
                populateOldSystemAscTrack = false;
                populateNewSystemTrack = false;
                
                okToPopulateOldSystemTrack = false;
                okToPopulateOldSystemAscTrack = false;

                okToDelOldSystemTrack = false;
                okToDelOldSystemAscTrack = false;       
                okToDelTrackFromOldSystem = false;
                okToDelTrackAscFromOldSystemAsc = false;
                    
                allSRx = uint8([BOSCH_SENSOR_SOURCE_local.RIGHTFRONT, ...
                        BOSCH_SENSOR_SOURCE_local.RIGHTREAR, ...
                        BOSCH_SENSOR_SOURCE_local.LEFTFRONT, ...
                        BOSCH_SENSOR_SOURCE_local.LEFTREAR]);
                
                %% Check trackIdx and trackAsc previos SystemTrack                                                  
                oldTrackAscId = uint8(0);                   
                oldTrackAscSource = uint8(0);  
                oldTrackAscInTable = uint8(0);
                lifeTimeoldAscTrack = double(0);
                oldTrackAscNewAscIdx = uint8(0);
                oldTrackAscNewAscSource = uint8(0);
                oldTrackAscNewAscSystemIdx = uint8(0);
                systemOldAscNewAscAge = double(0);
                lifeTimeoldAscNewAscTrack = double(0);

                if oldTrackSystemIdx ~= uint8(0) && oldTrackSystemIdx ~= oldTrackSystemAscIdx
                    % Was it alone?
                    oldTrackSystemIds = this.getSystemSourceId(oldTrackSystemIdx);
                    oldTrackSystemSources = this.getSystemSource(oldTrackSystemIdx);
         
                    if oldTrackSystemIds(1) == trackId &&...
                            oldTrackSystemSources(1) == trackSource
                        % Other track is oldTrackSystemIds(2)
                        oldTrackAscId = oldTrackSystemIds(2);
                        oldTrackAscSource = oldTrackSystemSources(2);
                        oldTrackAscInTable = this.isInTable(oldTrackAscSource, oldTrackAscId);
                    elseif oldTrackSystemIds(2) == trackId &&...
                            oldTrackSystemSources(2) == trackSource
                        oldTrackAscId = oldTrackSystemIds(1);
                        oldTrackAscSource = oldTrackSystemSources(1);
                        oldTrackAscInTable = this.isInTable(oldTrackAscSource, oldTrackAscId);
                    end
                    latPosOldAsc = single(0);
                    longPosOldAsc = single(0);
                    if oldTrackAscSource ~= 0 && oldTrackAscInTable ~= 0                  
                        lifeTimeoldAscTrack = double(BoschSensors.getObjectLifeTime(this, FLRdata.Signals, FLCdata.Signals, SRXdata, ...    
                            oldTrackAscSource, this.lookupTable(oldTrackAscInTable).signalNo));                                                                                           
                        latPosOldAsc = this.lookupTable(oldTrackAscInTable).closestSideLat;                              
                        longPosOldAsc = this.lookupTable(oldTrackAscInTable).closestSideLong;
                    end
                    
                    latPosOldAscNewAsc = single(0);
                    longPosOldAscNewAsc = single(0);
                    
                    if oldTrackAscInTable ~= 0  
                        oldTrackAscNewAscIdx = this.lookupTable(oldTrackAscInTable).associationPointer;
                        if oldTrackAscNewAscIdx ~= 0
                            oldTrackAscNewAscSource = this.lookupTable(oldTrackAscNewAscIdx).source;                  

                            if oldTrackAscNewAscSource == uint8(7)
                                oldTrackAscNewAscSystemIdx = this.lookupTable(oldTrackAscNewAscIdx).signalNo;
                            else                       
                                oldTrackAscNewAscSystemIdx = this.isMemberOfSystemTrack(oldTrackAscNewAscIdx, FLRdata, FLCdata, SRXdata);
                            end

                            if oldTrackAscNewAscSystemIdx ~= uint8(0)                           
                                systemOldAscNewAscAge = double(this.getSystemObjAge(oldTrackAscNewAscSystemIdx));
                                lifeTimeoldAscNewAscTrack = double(BoschSensors.getObjectLifeTime(this, FLRdata.Signals, FLCdata.Signals, SRXdata, ...    
                                    oldTrackAscNewAscSource, this.lookupTable(oldTrackAscNewAscIdx).signalNo));    
                                                                   
                                latPosOldAscNewAsc = this.lookupTable(oldTrackAscNewAscSystemIdx).closestSideLat;                              
                                longPosOldAscNewAsc = this.lookupTable(oldTrackAscNewAscSystemIdx).closestSideLong;
                            end
                        end
                    end

                    % Scenario 1: 
                    % If an SRX object is appeared which has broken the old FLR-FLR association in the (crossing) MOIS
                    % region, then FLR+SRX pair should inherit the old system track
                    if trackSource == this.sensorSources.FLR && oldTrackAscSource == this.sensorSources.FLR && ismember(trackAscSource, allSRx) && ...
                        ~ismember(oldTrackAscNewAscSource, allSRx) && 20 < atand(abs(longPosAsc/latPosAsc)) && atand(abs(longPosAsc/latPosAsc)) < 65
                        okToPopulateOldSystemTrack = true;
                        
                    % Scenario 2: 
                    % In (crossing) MOIS, if the main track is FLR with old FLR asc and the old FLR asc is now associated with an
                    % SRx object, and the current asc of the main FLR track is a new obj, the main track will not inherit the
                    % old system track 
                    elseif trackSource == this.sensorSources.FLR && oldTrackAscSource == this.sensorSources.FLR && ismember(oldTrackAscNewAscSource, allSRx) && ...
                            20 < atand(abs(longPosOldAscNewAsc/latPosOldAscNewAsc)) && atand(abs(longPosOldAscNewAsc/latPosOldAscNewAsc)) < 65 && oldTrackSystemAscIdx == uint8(0) && ...
                            oldTrackAscInTable ~= uint8(0)
                        okToDelTrackFromOldSystem = true;     
                        populateNewSystemTrack = true; 
                    
                    % Scenario 3:
                    % In (crossing) MOIS, if the previous association was FLR+SRx and now this association is split,
                    % SRx should inherit the ID 
                    % (part 1)
                    elseif oldTrackAscInTable ~= uint8(0) && ismember(trackSource, allSRx) && oldTrackAscSource == this.sensorSources.FLR && ...
                            ~(trackAscSource == oldTrackAscSource && trackAscId == oldTrackAscId) 
                        if 20 < atand(abs(longPos/latPos)) && atand(abs(longPos/latPos)) < 65
                            okToPopulateOldSystemTrack = true;
                        else
                            okToDelTrackFromOldSystem = true;
                            populateNewSystemTrack = true; 
                        end
  
                    % (part 2)
                    elseif oldTrackAscInTable ~= uint8(0) && trackSource == this.sensorSources.FLR && ismember(oldTrackAscSource, allSRx) && ...
                           ~(trackAscSource == oldTrackAscSource && trackAscId == oldTrackAscId)
                       if 20 < atand(abs(longPosOldAsc/latPosOldAsc)) && ...
                           atand(abs(longPosOldAsc/latPosOldAsc)) < 65 && oldTrackSystemAscIdx == uint8(0)
                            okToDelTrackFromOldSystem = true;     
                            populateNewSystemTrack = true;  
                       else
                           okToPopulateOldSystemTrack = true;
                       end
                    
                    % Scenario 4:
                    % in case of FLR-FLR association drop, the idea is to keep the system track for the FLR object with longer life time in case of new
                    % association - if they are same age, compare obstacleConf
                    elseif trackSource == this.sensorSources.FLR && oldTrackAscSource == this.sensorSources.FLR && oldTrackSystemAscIdx == uint8(0) && ...
                            oldTrackAscInTable ~= uint8(0) 
                        if lifeTimeTrack < lifeTimeoldAscTrack 
                            okToDelTrackFromOldSystem = true;     
                            populateNewSystemTrack = true;  
                        elseif lifeTimeTrack == lifeTimeoldAscTrack
                            obstacleConfTrack = BoschSensors.getObjectObstacleConf(this, FLRdata.Signals, SRXdata, trackSource, this.lookupTable(trackIdx).signalNo);
                            obstacleConfOldAscTrack = BoschSensors.getObjectObstacleConf(this, FLRdata.Signals, SRXdata, oldTrackAscSource, this.lookupTable(oldTrackAscInTable).signalNo);
                            if obstacleConfTrack < obstacleConfOldAscTrack
                                okToDelTrackFromOldSystem = true;     
                                populateNewSystemTrack = true; 
                            else                   
                                okToPopulateOldSystemTrack = true;
                            end
                        end  

                    % Scenario 5: (FLR conflict) if track source is not FLR and the asc is FLR but is a new object and the old asc
                    % was an FLR and we are not in the MOIS area, take a new SDF track 
                    elseif trackSource ~= this.sensorSources.FLR && oldTrackSystemAscIdx == uint8(0) && trackAscSource == this.sensorSources.FLR && ...
                            oldTrackAscSource == this.sensorSources.FLR && ~(ismember(trackSource, allSRx) && 20 < atand(abs(longPos/latPos)) && atand(abs(longPos/latPos)) < 65) && ...
                            oldTrackAscInTable ~= uint8(0) && (oldTrackAscId ~= trackAscId || oldTrackAscSource ~= trackAscSource) 
                        populateNewSystemTrack = true;
                        okToDelTrackFromOldSystem = true; 

                    % Scenario 6:    
                    % If both of the track source and its associate are FLR objects (without SRx object in MOIS area), activate case 2 or case 5                        
                    elseif (trackSource == this.sensorSources.FLR) && ~(ismember(trackAscSource, allSRx) && 20 < atand(abs(longPosAsc/latPosAsc)) && atand(abs(longPosAsc/latPosAsc)) < 65) || ...
                           (trackAscSource == this.sensorSources.FLR) && ~(ismember(trackSource, allSRx) && 20 < atand(abs(longPos/latPos)) && atand(abs(longPos/latPos)) < 65)
                        okToPopulateOldSystemTrack = true;
                        okToPopulateOldSystemAscTrack = true;
                         
                    % Scenario 7:    
                    % If ascObject is not empty and is still in the lookupTable and is different from the current ascObject,
                    % keep the same system track for the sensor with longer life time
                    elseif oldTrackAscSource ~= 0 && oldTrackAscInTable ~= uint8(0) && (oldTrackAscId ~= trackAscId || oldTrackAscSource ~= trackAscSource) && ...
                            lifeTimeTrack < lifeTimeoldAscTrack 
                        populateNewSystemTrack = true;
                        okToDelTrackFromOldSystem = true;

                    % Scenario 8:
                    % If empty, or ascObject not reported anymore delete this track if not populated
                    elseif oldTrackAscSource == uint8(0) || oldTrackAscInTable == uint8(0)
                        okToPopulateOldSystemTrack = true;
                        okToDelOldSystemTrack = true;
                    
                    % Scenario 9:
                    % If the main track is not an FLR object and does not have an association and the old asc was FLR and it's still in the table and we are not in the MOIS region, then
                    % FLR object should inherit the SDF ID
                    elseif trackSource ~= this.sensorSources.FLR && oldTrackAscSource == this.sensorSources.FLR && oldTrackSystemAscIdx == uint8(0) && ...
                            oldTrackAscInTable ~= uint8(0)
                        if ~(ismember(trackSource, allSRx) && 20 < atand(abs(longPos/latPos)) && atand(abs(longPos/latPos)) < 65) && ...
                                (oldTrackAscId ~= trackAscId || oldTrackAscSource ~= trackAscSource) 
                            populateNewSystemTrack = true; 
                            okToDelTrackFromOldSystem = true;
                        else
                            okToPopulateOldSystemTrack = true;
                        end                    
                        
                    % Scenario 10:
                    % If not empty, populate ok, but not delete whole track
                    else    
                        okToPopulateOldSystemTrack = true;
                        okToDelTrackFromOldSystem = true;
                    end               
                end
                
                %% Check trackAscIdx old partner
                if oldTrackSystemAscIdx ~= uint8(0) && oldTrackSystemIdx ~= oldTrackSystemAscIdx
                    % Was it alone?                    
                    oldTrackSystemAscIds = this.getSystemSourceId(oldTrackSystemAscIdx);
                    oldTrackSystemAscSources = this.getSystemSource(oldTrackSystemAscIdx);

                    oldTrackAscAscId = uint8(0);
                    oldTrackAscAscSource = uint8(0);
                    oldTrackAscAscInTable = uint8(0);
                    
                    if oldTrackSystemAscIds(1) == trackAscId &&...
                            oldTrackSystemAscSources(1) == trackAscSource
                        % Other track is oldTrackSystemIds(2)
                        oldTrackAscAscId = oldTrackSystemAscIds(2);
                        oldTrackAscAscSource = oldTrackSystemAscSources(2);
                        oldTrackAscAscInTable = this.isInTable(oldTrackAscAscSource, oldTrackAscAscId);

                    elseif oldTrackSystemAscIds(2) == trackAscId &&...
                            oldTrackSystemAscSources(2) == trackAscSource
                        oldTrackAscAscId = oldTrackSystemAscIds(1);
                        oldTrackAscAscSource = oldTrackSystemAscSources(1);
                        oldTrackAscAscInTable = this.isInTable(oldTrackAscAscSource, oldTrackAscAscId);
                    end
                    
                    if oldTrackAscAscSource ~= 0 && oldTrackAscAscInTable ~= 0                  
                        lifeTimeoldAscAscTrack = double(BoschSensors.getObjectLifeTime(this, FLRdata.Signals, FLCdata.Signals, SRXdata, ...    
                            oldTrackAscAscSource, this.lookupTable(oldTrackAscAscInTable).signalNo));
                    end
                                            
                    % Scenario 11: Check if ascossiated is coasted
                    if trackAscSource == this.sensorSources.SDF
                        okToPopulateOldSystemAscTrack = true;
                        okToDelOldSystemAscTrack = true;
                     
                    % Scenario 12: If FLR and SRX are just being fused and each have
                    % their own SDF ID and at least one has good objClass, which ID should we continue with? the one
                    % that has the good objClass. In case both have good objClass the one with objClassSource == FLC should
                    % dominate, otherwise just keep the older one
                    elseif ((trackSource == this.sensorSources.FLR && ismember(trackAscSource, allSRx)) || ...
                            (trackAscSource == this.sensorSources.FLR && ismember(trackSource, allSRx))) && oldTrackSystemAscIdx ~= uint8(0) && oldTrackAscAscInTable == uint8(0) && ...
                            oldTrackAscInTable == uint8(0) && (objClassTrack == uint8(4) || objClassTrack == uint8(5) || objClassAscTrack == uint8(4) || objClassAscTrack == uint8(5))
                            if (objClassTrack == uint8(4) || objClassTrack == uint8(5)) && ~(objClassAscTrack == uint8(4) || objClassAscTrack == uint8(5))
                                okToPopulateOldSystemTrack = true;
                                okToPopulateOldSystemAscTrack = false;
                            elseif (objClassAscTrack == uint8(4) || objClassAscTrack == uint8(5)) && ~(objClassTrack == uint8(4) || objClassTrack == uint8(5))
                                okToPopulateOldSystemAscTrack = true;  
                                okToPopulateOldSystemTrack = false;
                            elseif objClassSourceTrack == this.sensorSources.FLC && objClassAscSourceTrack ~= this.sensorSources.FLC
                                 okToPopulateOldSystemTrack = true;
                                 okToPopulateOldSystemAscTrack = false;
                            elseif objClassSourceTrack ~= this.sensorSources.FLC && objClassAscSourceTrack == this.sensorSources.FLC
                                 okToPopulateOldSystemAscTrack = true;
                                 okToPopulateOldSystemTrack = false;
                            elseif systemAge > systemAscAge
                                okToPopulateOldSystemTrack = true;
                                okToPopulateOldSystemAscTrack = false;
                            else
                                okToPopulateOldSystemAscTrack = true;
                                okToPopulateOldSystemTrack = false;
                            end

                    % Scenario 13 (4 parts): if we have FLR+SRx new fusion in the transition area and system ages agree, prioritize SRx. Ow, prioritize FLR (after FLR dominancy check)                       
                    elseif oldTrackSystemIdx ~= oldTrackSystemAscIdx && trackSource == this.sensorSources.FLR && ...
                            ismember(trackAscSource, allSRx) && 20 < atand(abs(longPosAsc/latPosAsc)) && atand(abs(longPosAsc/latPosAsc)) < 65 && ...
                            systemAge < systemAscAge
                        okToPopulateOldSystemTrack = false;
                        okToPopulateOldSystemAscTrack = true;

                    % (part 2)
                    elseif oldTrackSystemIdx ~= oldTrackSystemAscIdx && trackAscSource == this.sensorSources.FLR && ...
                            ismember(trackSource, allSRx) && 20 < atand(abs(longPos/latPos)) && atand(abs(longPos/latPos)) < 65 && oldTrackSystemIdx ~= uint8(0) && ...
                            systemAge > systemAscAge
                        okToPopulateOldSystemTrack = true;
                        okToPopulateOldSystemAscTrack = false;
                        
                    % Scenario 14: If FLR dominancy exists without a competition between the old associates of the main
                    % track and the associate track, then we would just consider the main and the associate track and not the
                    % old associate (case 5.3)
                    elseif ((trackSource == this.sensorSources.FLR && trackAscSource == this.sensorSources.FLR) && ...
                            (oldTrackAscAscSource ~= this.sensorSources.FLR || oldTrackAscAscInTable == uint8(0)) && ...
                            (oldTrackAscSource ~= this.sensorSources.FLR || oldTrackAscInTable == uint8(0))) || ...
                            ((trackAscSource == this.sensorSources.FLR && (oldTrackAscAscSource ~= this.sensorSources.FLR || oldTrackAscAscInTable == uint8(0))) && ...
                            (trackSource == this.sensorSources.FLR && (oldTrackAscSource ~= this.sensorSources.FLR || oldTrackAscInTable == uint8(0))))                      
                        okToPopulateOldSystemTrack = true;
                        okToPopulateOldSystemAscTrack = true;  
                  
                    % Scenario 15: Handling FLR conflicts - probably needs
                    % checking if oldTrackAscAscInTable ~= uint8(0) and/or
                    % oldTrackAscInTable ~= uint8(0) (in the first if statement)
                    elseif oldTrackSystemIdx ~= oldTrackSystemAscIdx && (trackSource == this.sensorSources.FLR && trackAscSource == this.sensorSources.FLR)
                        if (oldTrackAscAscSource == this.sensorSources.FLR && lifeTimeoldAscAscTrack > lifeTimeTrack) && (oldTrackAscSource == this.sensorSources.FLR && lifeTimeoldAscTrack > lifeTimeAscTrack)
                            populateNewSystemTrack = true;
                            okToPopulateOldSystemTrack = false;
                            okToPopulateOldSystemAscTrack = false;  
                        elseif (oldTrackAscAscSource == this.sensorSources.FLR && lifeTimeoldAscAscTrack > lifeTimeTrack)                           
                            okToPopulateOldSystemTrack = true;
                            okToPopulateOldSystemAscTrack = false;
                        elseif (oldTrackAscSource == this.sensorSources.FLR && lifeTimeoldAscTrack > lifeTimeAscTrack)                                        
                            okToPopulateOldSystemTrack = false;
                            okToPopulateOldSystemAscTrack = true;
                        else
                            okToPopulateOldSystemTrack = true;
                            okToPopulateOldSystemAscTrack = true; 
                        end 

                    % (Scenario 13 - part 3)    
                    elseif oldTrackSystemIdx ~= oldTrackSystemAscIdx && (trackSource == this.sensorSources.FLR && ismember(trackAscSource, allSRx)) && oldTrackSystemIdx ~= uint8(0)
                        okToPopulateOldSystemTrack = true;
                        okToPopulateOldSystemAscTrack = false;        
                        
                    % (Scenario 13 - part 4)
                    elseif oldTrackSystemIdx ~= oldTrackSystemAscIdx && (trackAscSource == this.sensorSources.FLR && ismember(trackSource, allSRx))  
                        okToPopulateOldSystemTrack = false;
                        okToPopulateOldSystemAscTrack = true;

                    % Scenario 16: if the main track is FLR and the
                    % ascTrack is not FLR and there is no FLR competitor,
                    % then the main track should keep the ID ( systemAge > 20 part is to handle the cases when correct association does not happen instantly)
                    elseif trackSource == this.sensorSources.FLR && oldTrackAscSource ~= this.sensorSources.FLR && trackAscSource ~= this.sensorSources.FLR && ...
                            oldTrackAscAscSource ~= this.sensorSources.FLR && systemAge > 20
                        okToPopulateOldSystemTrack = true;
                        okToPopulateOldSystemAscTrack = false;

                    % Scenario 17: if a new sensor track is being
                    % evaluated and the oldascasc is FLR and in the table while asc is
                    % not FLR and we are not in MOIS area, give
                    % priority to oldascasc
                    elseif oldTrackSystemIdx == 0 && trackAscSource ~= this.sensorSources.FLR && oldTrackAscAscSource == this.sensorSources.FLR && ...
                            oldTrackAscAscInTable ~= uint8(0) && ~(ismember(trackAscSource, allSRx) && 20 < atand(abs(longPosAsc/latPosAsc)) && atand(abs(longPosAsc/latPosAsc)) < 65)   
                        populateNewSystemTrack = true;
                        okToDelTrackAscFromOldSystemAsc = true;

                    % Scenario 18: Comparing the life time of the new associated couples
                    % (the current and its associate track) with 
                    elseif (max(lifeTimeTrack, lifeTimeAscTrack) > max(lifeTimeoldAscTrack, lifeTimeoldAscAscTrack) && systemAge < systemAscAge) || ...
                            (lifeTimeoldAscTrack > max(lifeTimeTrack, lifeTimeAscTrack) &&  systemAge > systemAscAge)  || ...
                             (lifeTimeoldAscAscTrack > max(lifeTimeTrack, lifeTimeAscTrack) && systemAge > systemAscAge)
                        okToPopulateOldSystemTrack = false;
                        okToPopulateOldSystemAscTrack = true;   
                        
                    % Scenario 19: If the old and new asc tracks are not the same, then the
                    % sensor pairs with longer life time would win the SDF ID with older system age
                    elseif oldTrackAscInTable ~= uint8(0) && trackAscIdx ~= oldTrackAscInTable && ...
                            oldTrackAscNewAscIdx ~= uint8(0) && oldTrackAscNewAscSystemIdx ~= uint8(0) && ...
                             max(lifeTimeoldAscTrack, lifeTimeoldAscNewAscTrack) > max(lifeTimeTrack, lifeTimeAscTrack) && systemAge > systemOldAscNewAscAge
                            okToPopulateOldSystemTrack = false; 
                            okToDelTrackFromOldSystem = true;
                            
                    % Scenario 20    
                    elseif oldTrackAscInTable ~= uint8(0) && trackAscIdx ~= oldTrackAscInTable && ...
                            oldTrackAscNewAscIdx ~= uint8(0) && oldTrackAscNewAscSystemIdx == uint8(0) && ...
                            lifeTimeoldAscTrack > max(lifeTimeTrack, lifeTimeAscTrack)                                                                              
                        okToPopulateOldSystemTrack = false;    
                        okToDelTrackFromOldSystem = true;
                        
                    % Scenario 21: Check if the oldAsc source of the trackAsc is FLR but the associate track is not, then
                    % it should inherit the ID
                    elseif oldTrackAscAscSource == this.sensorSources.FLR && ...
                            oldTrackAscAscInTable ~= uint8(0) && trackAscSource ~= this.sensorSources.FLR
                        okToPopulateOldSystemAscTrack = false;
                        okToDelTrackAscFromOldSystemAsc = true;
                        
                    % Scenario 22: If empty, delete this track if not populated
                    elseif oldTrackAscAscSource == uint8(0) || oldTrackAscAscInTable == uint8(0)
                        okToPopulateOldSystemAscTrack = true;
                        okToDelOldSystemAscTrack = true;
                        
                    % Scenario 23: If not empty, populate ok, but not delete whole track
                    else 
                        okToPopulateOldSystemAscTrack = true;
                        okToDelTrackAscFromOldSystemAsc = true;
                    end
                end
     
                %% Select which systemTrack to populate           
                % Case 0: If association is coasted track, keep the coasted
                % system track
                if (trackAscSource == this.sensorSources.SDF && oldTrackSystemIdx ~= uint8(0)) || (trackSource == this.sensorSources.SDF && oldTrackSystemAscIdx ~= uint8(0))
                    systemAge = this.getSystemObjAge(oldTrackSystemIdx);
                    systemAscAge = this.getSystemObjAge(oldTrackSystemAscIdx);
                    if systemAge >= systemAscAge
                        populateOldSystemTrack = true;
                    elseif systemAge <= systemAscAge
                        populateOldSystemAscTrack = true;
                        okToDelTrackFromOldSystem = true;
                    end            
                
                % Case 1: None of the tracks exist previously
                elseif oldTrackSystemIdx == uint8(0) && oldTrackSystemAscIdx == uint8(0)
                    populateNewSystemTrack = true;
                    
                % Case 2:
                % Same association as last time
                elseif oldTrackSystemIdx == oldTrackSystemAscIdx
                    populateOldSystemTrack = true;
                    % As both are from same systemTrack, delete nothing
                    okToDelOldSystemTrack = false;
                    okToDelOldSystemAscTrack = false;
                    okToDelTrackFromOldSystem = false;
                    okToDelTrackAscFromOldSystemAsc = false;
                    
                % Case 3:
                % This track existed previosly but not associationTrack
                elseif oldTrackSystemIdx ~= uint8(0) && oldTrackSystemAscIdx == uint8(0)
                    if okToPopulateOldSystemTrack == true
                        populateOldSystemTrack = true;
                    else
                        populateNewSystemTrack = true;
                    end

                % Case 4: 
                % Association track did exist previosly but not "this"
                % track.
                elseif oldTrackSystemIdx == uint8(0) && oldTrackSystemAscIdx ~= uint8(0)
                    if okToPopulateOldSystemAscTrack == true
                        populateOldSystemAscTrack = true;
                    else
                        populateNewSystemTrack = true;
                    end
                % Case 5: 
                % Where both tracks did exit in previous system. If both
                % are previosly in same systemTrack, it will be caught in
                % Case 2. 
                % Who should be populated now?
                elseif oldTrackSystemIdx ~= uint8(0) && oldTrackSystemAscIdx ~= uint8(0)
                    % Case 5.1 Ok to populate SystemTrack but not Asc
                    if okToPopulateOldSystemTrack == true && ...
                            okToPopulateOldSystemAscTrack == false
                        populateOldSystemTrack = true;
                    % Case 5.2 Ok to populate SystemAscTrack but not
                    % systemTrack
                    elseif okToPopulateOldSystemTrack == false && ...
                            okToPopulateOldSystemAscTrack == true
                        populateOldSystemAscTrack = true;
                    % Case 5.3 Both are ok to populate.
                    elseif okToPopulateOldSystemTrack == true && ...
                            okToPopulateOldSystemAscTrack == true
                        % Check for FLR object and keep its track.
                        % If both are FLR, pick the longest age.
                        if trackSource == this.sensorSources.FLR && ...
                                trackAscSource ~= this.sensorSources.FLR
                            populateOldSystemTrack = true;
                            okToDelTrackAscFromOldSystemAsc = true;
                        elseif trackSource ~= this.sensorSources.FLR && ...
                                trackAscSource == this.sensorSources.FLR
                            populateOldSystemAscTrack = true;
                            okToDelTrackFromOldSystem = true;
                        else
                            systemAge = this.getSystemObjAge(oldTrackSystemIdx);
                            systemAscAge = this.getSystemObjAge(oldTrackSystemAscIdx);
                            if systemAge >= systemAscAge
                                populateOldSystemTrack = true;
                                okToDelTrackAscFromOldSystemAsc = true;
                            elseif systemAge <= systemAscAge
                                populateOldSystemAscTrack = true;
                                okToDelTrackFromOldSystem = true;
                            end
                        end
                    % Case 5.4 None of the old places are ok
                    else
                        populateNewSystemTrack = true;
                    end
                end
                
                %% Fuse into a systemTrack
                % First, check which systemTrack to populate, old, AscOld
                % or a new one.
                % Secondly, check criterias for deletion. Whole systemTrack
                % or just delete a sensorTrack from a systemTrack?
                if trackSource == this.sensorSources.SDF || trackAscSource == this.sensorSources.SDF
                    % Special handling when fusing coasted objects
                    % Remove the coasted object but not if the real track
                    % does not have a systemIdx, then inherit it.
                    % fuseSystemTrack will handle and remove the coasted
                    % track.
                    % As isMemberOfSystemTrack will not return Idx to
                    % coasted object, it has to be done "manually"
                    if oldTrackSystemIdx ~= uint8(0) && oldTrackSystemAscIdx == uint8(0)
                        % If the real track already exist, populate that
                        % systemTrack if ok. Else, populate coastedTrack
                        if okToPopulateOldSystemTrack
                            this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, oldTrackSystemIdx, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                        else
                            this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, this.lookupTable(trackAscIdx).signalNo, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                        end
                    elseif oldTrackSystemIdx == uint8(0) && oldTrackSystemAscIdx ~= uint8(0)
                        % If the real track already exist, populate that
                        % systemTrack if ok
                        if okToPopulateOldSystemAscTrack
                            this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, oldTrackSystemAscIdx, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                        else
                            this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, this.lookupTable(trackIdx).signalNo, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                        end
                    else
                        if trackSource == this.sensorSources.SDF && trackAscSource == this.sensorSources.SDF
                            % Two coasted objects, fuse by age
                            systemAge = this.getSystemObjAge(this.lookupTable(trackIdx).signalNo);
                            systemAscAge = this.getSystemObjAge(this.lookupTable(trackAscIdx).signalNo);
                            if systemAge >= systemAscAge
                                this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, this.lookupTable(trackIdx).signalNo, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                            elseif systemAge <= systemAscAge
                                this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, this.lookupTable(trackAscIdx).signalNo, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                            end
                        elseif trackAscSource == this.sensorSources.SDF 
                            if populateOldSystemAscTrack == 1 % to avoid a random coasted object stealing the ID  
                                this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, this.lookupTable(trackAscIdx).signalNo, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                                if okToDelOldSystemTrack  
                                    % The track has now been fused with a previously coasted track, so it needs to
                                    % be deleted.
                                    this.deleteSystemTrack(oldTrackSystemIdx);
                                elseif okToDelTrackFromOldSystem == true   
                                    this.deleteSystemTrackSource(oldTrackSystemIdx, trackSource, trackId);
                                end
                            else                                  
                                this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, oldTrackSystemIdx, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                            end
                        else
                            this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, this.lookupTable(trackIdx).signalNo, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                        end
                    end
                elseif populateOldSystemTrack == true
                    this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, oldTrackSystemIdx, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                    
                    if okToDelOldSystemAscTrack == true
                        this.deleteSystemTrack(oldTrackSystemAscIdx);
                    elseif okToDelTrackAscFromOldSystemAsc == true
                        this.deleteSystemTrackSource(oldTrackSystemAscIdx, trackAscSource, trackAscId);
                    end
                elseif populateOldSystemAscTrack == true
                    this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, oldTrackSystemAscIdx, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                    
                    if okToDelOldSystemTrack == true
                        this.deleteSystemTrack(oldTrackSystemIdx);
                    elseif okToDelTrackFromOldSystem == true
                        this.deleteSystemTrackSource(oldTrackSystemIdx, trackSource, trackId);
                    end
                elseif populateNewSystemTrack == true
                    this.fuseSystemTrack(fusionCycleTimeMs, trackIdx, trackAscIdx, uint8(0), FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                    
                    if okToDelOldSystemTrack == true
                        this.deleteSystemTrack(oldTrackSystemIdx);
                    elseif okToDelTrackFromOldSystem == true
                        this.deleteSystemTrackSource(oldTrackSystemIdx, trackSource, trackId);
                    end
                    
                    if okToDelOldSystemAscTrack == true
                        this.deleteSystemTrack(oldTrackSystemAscIdx);
                    elseif okToDelTrackAscFromOldSystemAsc == true
                        this.deleteSystemTrackSource(oldTrackSystemAscIdx, trackAscSource, trackAscId );
                    end
                end
            end
            
            %% COASTING
            %Loop though all systemTracks that was not updated from a new
            [oldSystemIdx , dAge] = this.getAllOldSystemTracks();
            for i = 1 : this.N_OBJ_TRACKS
                % Only coast objects that are older than listTime
                if oldSystemIdx(i) == 0 || dAge(i) == 0
                    continue;
                end
                
                % TODO use existanceProbability instead of than objClass conf
                % or longPosVar ?
                %                 confidence = this.getSystem CONFIDENCE!!!
                conf = this.getSystemObjClassConf(i);
                age = this.getSystemObjAge(i);
                
                vehicleWidth = single(single(parameters.P1E50_VehicleWidth_mm_v) / 1000);
                criticalLongDistance = single(10);

                latPos = this.getSystemLatPos(i);
                longPos = this.getSystemLongPos(i);

                % This is absolute velocities
                latVel = this.getSystemLatVel(i);
                longVel = this.getSystemLongVel(i);
                
                absVel = sqrt(latVel.^2 + longVel.^2);

                objClass = this.getSystemObjClass(i);
                
                source = this.getSystemObjSource(i);
                                
                maxLongDistBSIS = single(7);
                minLongDistBSIS = single(-30);
                

                % Check for MOIS area, max speed and object class
                if 0 < longPos && longPos < criticalLongDistance && ...
                        -vehicleWidth/2 - 7 < latPos && latPos < vehicleWidth/2 + 7 && ...
                            absVel > 0.1 && absVel < 5 && ...
                                (objClass == 0 || objClass == 1 || objClass == 4 || objClass == 5 || objClass == 8 || objClass == 9 || objClass == 15) && ...
                                    (source(1) ~= this.sensorSources.RIGHTREAR && source(1) ~= this.sensorSources.LEFTREAR && ...
                                        source(2) ~= this.sensorSources.RIGHTREAR && source(2) ~= this.sensorSources.LEFTREAR)
                    coastingTimeLimit = single(3000);
                    coastLimit = single(3000);
                % Check for BSIS area and object class
                elseif longPos < maxLongDistBSIS && longPos > minLongDistBSIS && ...
                        -vehicleWidth/2 - 5 < latPos && latPos < vehicleWidth/2 + 5 && ...
                            (objClass == 0 || objClass == 4 || objClass == 5 || objClass == 15) && ...
                                (source(1) ~= this.sensorSources.FLR && source(1) ~= this.sensorSources.FLC) && ...
                                (source(2) ~= this.sensorSources.FLR && source(2) ~= this.sensorSources.FLC) 
                   coastingTimeLimit = single(1000);
                   coastLimit = single(1000);
                else
                    coastingTimeLimit = calculateDynamicCoastingTime(age, conf);
                    coastLimit = single(MAX_COASTING);
                end
                
                objectCoastingTime = this.getSystemCoastingTime(i);
                
                if ((objectCoastingTime > coastLimit ) || (objectCoastingTime >= coastingTimeLimit)) && ...
                        oldSystemIdx(i) ~= 0
                    this.deleteSystemTrack(oldSystemIdx(i));
                    
                elseif dAge(i) > 0 && oldSystemIdx(i) ~= 0
                    % Check if the sensorTracks in oldSystemIdx
                    % exist in other tracks. If yes then delete track
                    % instead
                    existSystemTrack = findSensorTrackInSystemTrack(this, oldSystemIdx(i), allOldSystemTrack, oldLookupTable);
                    
                    if existSystemTrack == 0
                        this.coastSystemTrack(oldSystemIdx(i), fusionCycleTimeMs, FLRdata, FLCdata, SRXdata, MotionEstimatorData, EgoVehicleData, dacuTime, parameters);
                    else
                        % Delete the track if it alreday exist 
                        % systemTrack as updated
                        this.deleteSystemTrack(oldSystemIdx(i));
                    end
                end
            end
        end
        
