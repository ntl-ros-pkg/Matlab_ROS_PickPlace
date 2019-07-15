function robot = loadRobotFromURDF(urdf_filename)
    %loadRobotFromURDF
    
    urdf = xmlread(urdf_filename);
    robotElement = urdf.getElementsByTagName('robot').item(0);
    if isempty(robotElement)
        robotElement = urdf.getElementsByTagName('model').item(0);
    end
    
    % clear all the gazebo elements
    gazeboElements = urdf.getElementsByTagName('gazebo');
    cnt = 0;
    
    while gazeboElements.getLength() > 0
        gazeboElements = robotElement.getElementsByTagName('gazebo');
        gazeboElements.item(0).getAttribute('reference')
        %disp('-----');
        ge = gazeboElements.item(0);
        cnt = cnt + 1;
        robotElement.removeChild(ge);
        gazeboElements.getLength()
        %disp('--------------------');
    end

    
    if isempty(robotElement)
       error('There are no robots in [%s]', urdf_filename);
    end
    
    robotname = char(robotElement.getAttribute('name'));
    robotname = matlab.lang.makeValidName(robotname);
    fprintf('Robot name is: %s \n\n', robotname);
    
    robot = robotics.RigidBodyTree();
    
    %% links
    linkElements = robotElement.getElementsByTagName('link');
    lnum = linkElements.getLength();
        
    bodies = {};
    
    for i=0:(lnum-1)
        lname = char(linkElements.item(i).getAttribute('name'));
        lname = matlab.lang.makeValidName(lname);
        body = robotics.RigidBody(lname);
        
        %visualElement = linkElements.item(i).getElementsByTagName('visual').item(0);
        visualElement = linkElements.item(i).getElementsByTagName('collision').item(0); % use collision mesh
        if ~isempty(visualElement)
            fprintf('-- %3d - %s - (V)', i, lname);
            geoElement = visualElement.getElementsByTagName('geometry').item(0);
            geoChildren = geoElement.getChildNodes();
            for j = 1:geoChildren.getLength()
                gc = geoChildren.item(j-1);
                switch(lower(char(gc.getNodeName() )))
                    case 'mesh'
                        meshfilename = char(gc.getAttribute('filename'));
                        meshfilename = strrep(meshfilename,'/',filesep);
                        meshfilename = strrep(meshfilename,'\',filesep);  
                        
                        if ~isempty(strfind(meshfilename, ['package:',filesep,filesep]))
                            meshfilename=strrep(meshfilename,['package:',filesep,filesep],'');
                        end
                        
                        %meshfilename = strrep(meshfilename, '.DAE', '.STL');
                        [~, fn, ext] = fileparts(meshfilename);
                        meshfilename = fullfile([robotname '_mesh'], [fn ext]);
                        fprintf(' - %s', meshfilename);
                        
                        % cs: body.MeshFileName = meshfilename; % MeshFileName
                    case 'box'
                        boxdimstr = char(gc.getAttribute('size'));
                        boxdim = strCellArray2Mat(strsplit(strtrim(boxdimstr)));
                        meshfilename = fullfile([robotname '_mesh'], [body.Name '_coll.stl']);
                        boxMeshGen(boxdim, meshfilename);
                        fprintf(' - %s (auto-generated from box)', meshfilename);
                        body.MeshFileName = meshfilename;
                end
            end
                        
            origElement = visualElement.getElementsByTagName('origin').item(0);
            xyz = zeros(1,3); rpy = zeros(1,3);
        
            if ~isempty(origElement)
                if origElement.hasAttribute('xyz')
                    s = char(origElement.getAttribute('xyz'));
                    xyz = strCellArray2Mat(strsplit(strtrim(s)));
                end
                if origElement.hasAttribute('rpy')
                    s = char(origElement.getAttribute('rpy')); 
                    rpy = strCellArray2Mat(strsplit(strtrim(s)));
                end
            end           
            if isequal(xyz, zeros(1,3)) && isequal(rpy, zeros(1,3))
                fprintf('[x]'); 
            end
            
            ypr = [rpy(3) rpy(2) rpy(1)];
            R = eul2rotm(ypr, 'ZYX');
%           cs: body.Tvis = [R, xyz';[0 0 0], 1]; % Tvis
            
            fprintf('\n');
            
        else
            fprintf('-- %3d - %s (no visual)\n', i, lname);
        end
        bodies{end+1} = body; 
    end
    
    parents = repmat({robotics.RigidBody.empty}, 1, lnum);
    children_sets = repmat({ {} }, 1, lnum);

    
    %% joints
    fprintf('-----\n\n');
    jointElements = robotElement.getElementsByTagName('joint');
    fprintf('-- %3s - %25s  %25s ---> %-25s --- %s\n\n', 'Id', 'joint name', ...
             'parent link', 'child link', 'type');    
    for i=0:(jointElements.getLength()-1)
        jointElement = jointElements.item(i);
        
        % parent, child, joint type
        p = jointElement.getElementsByTagName('parent').item(0);
        if isempty(p) % Then it's not the main joint element. For instance, the transmission element
            continue;
        end
        c = jointElement.getElementsByTagName('child').item(0);
        pname = char(p.getAttribute('link'));
        pname = matlab.lang.makeValidName(pname);
        cname = char(c.getAttribute('link'));
        cname = matlab.lang.makeValidName(cname);
        jointElement.getAttribute('name')
        [parent, kp] = findLinkByName(bodies, pname);
        [child, kc] = findLinkByName(bodies, cname);
        
        jtype = jointElement.getAttribute('type'); % joint type
        if strcmp(jtype, 'continuous')
            jtype = 'revolute';
        end
        jname = jointElement.getAttribute('name'); % joint name
        jnt = robotics.Joint(matlab.lang.makeValidName(char(jname)), char(jtype));
        
        
        % FixedTransformation
        xyz = zeros(3,1); rpy = zeros(3,1);
        origin = jointElement.getElementsByTagName('origin').item(0);
        if ~isempty(origin)
            if origin.hasAttribute('xyz')
                s = char(origin.getAttribute('xyz'));
                xyz = strCellArray2Mat(strsplit(strtrim(s)));
            end
            if origin.hasAttribute('rpy')
                s = char(origin.getAttribute('rpy')); 
                rpy = strCellArray2Mat(strsplit(strtrim(s)));
            end
        end
        
        %R = rpy2rotm(rpy);
        ypr = [rpy(3) rpy(2) rpy(1)];
        R = eul2rotm(ypr, 'ZYX');
        jnt.setFixedTransform([R, reshape(xyz, 3,1);[0 0 0], 1]); % joint FixedBodyTransform
        
        limitElement = jointElement.getElementsByTagName('limit').item(0);
        if ~isempty(limitElement) && ~strcmp(jnt.Type,'fixed')
            lb = str2double(char(limitElement.getAttribute('lower')));
            if isnan(lb)
                lb = jnt.PositionLimits(1);
            end
            ub = str2double(limitElement.getAttribute('upper'));
            if isnan(ub)
                ub = jnt.PositionLimits(2);
            end
            jnt.PositionLimits = [lb, ub];
            jnt.HomePosition = (lb+ub)/2;
        end
                
        if strcmp(jtype, 'revolute') || strcmp(jtype, 'prismatic')
            axisElement = jointElement.getElementsByTagName('axis').item(0); % joint axis
            if ~isempty(axisElement)
                s = char(axisElement.getAttribute('xyz'));
                v = strCellArray2Mat(strsplit(strtrim(s)));
                jnt.JointAxis = v;
                %if norm(v-[ 0 0 1])> 1e8
                %    error('joint axis has to be [0 0 1]')
                %end
            end   
        end
        
        child.Joint = jnt;
        parents{kc} = parent;
        children_sets{kp}{end+1} = child;
        
        
        % summary text
        fprintf('-- %3d - %25s  %25s ---> %-25s --- %s\n', i, jnt.Name, ...
             parent.Name, child.Name, jnt.Type);
         

    end

    % generate sequence
    for i = 1:lnum
        if isempty(parents{i})
            fprintf('The hub body is %s [%d] \n',bodies{i}.Name, i);
            break
        end
    end
    
    
    robot.BaseName = bodies{i}.Name;

    bodies2add = children_sets{i}; % {bodies{i}};
    while ~isempty(bodies2add)
        body = bodies2add{1};
        [~, k] = findLinkByName(bodies, body.Name);
        robot.addBody(body, parents{k}.Name);
        bodies2add = {bodies2add{2:end}};
        bodies2add = [children_sets{k}, bodies2add];
    end
   
end


function [link, k] = findLinkByName(links, linkname)
    link = robotics.RigidBody.empty;
    k = 1;
    while k <= length(links)
        if ~strcmp(links{k}.Name, linkname)
            k = k + 1;
        else
            link = links{k};
            break;
        end
    end
    if k > length(links)
        error('could not find a match for %s', linkname);
    end
end



% function showGraph(links)
%     s = {};
%     t = {};
%     hl = zeros(1, length(links));
% 
%     for i = 1: length(links)      
%         p = links(i).ParentId;
%         if isempty(p)
%             s{end+1} = 'world';
%         else
%             %s{end+1} = char(links(i).ParentName); 
%             s{end+1} = sprintf('%d-%s', links(i).ParentId, links(i).ParentName);
%         end
%         %t{end+1} = char(links(i).Name); 
%         t{end+1} = sprintf('%d-%s',links(i).Id, links(i).Name);
%         if strcmp(lower(links(i).JointType), 'revolute')
%             hl(i) = 1; 
%         end
%     end
%     hp = plot(graph(s,t));
%     hp.NodeColor = 'b';
%     hp.Marker = 's';
%     for i = 1:length(links)
%         if hl(i) == 1
%             highlight(hp, [links(i).ParentId+1, i+1], 'EdgeColor', 'r','LineWidth',2); % need to plus 1 here
%         end
%     end
%     
%     set(gcf,'color','w');
%     set(gca,'visible','off');
% end

% s is a string cell array
function A = strCellArray2Mat(s)
    [m, n] = size(s);
    A = zeros(m,n);
    for i = 1:m
        for j = 1:n
            A(i,j) = str2double(s{i,j});
        end
    end
    
end

