function [condition] = testifcom(comstr)
    current_COMS = list_serialports();
    condition = 0;
    for i=1:length(current_COMS)
        if strncmp(comstr,current_COMS{i},length(comstr)) == 1
            condition = 1;
            break;
        end
    end
