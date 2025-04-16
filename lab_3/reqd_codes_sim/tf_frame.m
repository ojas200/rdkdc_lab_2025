% Class to create and maintain a frame in tf 
% communicate with ros through topic matlab_frame

% Author: Mengze Xu, 09/30/2017
% Updated by: Jakub Piwowarczyk, 08/27/2023
% Timeout Functionality Updated by: Zuriel Joven, 03/18/2024

classdef tf_frame < handle
    
    properties (SetAccess = protected)
        frame_name  % frame name
        base_frame_name % reference frame name
        pose % g 4*4 matrix
        %tftree % same as ur5_interface, it's better to use one tftree to lookup, but just for the course, take it easy.
    end

    methods (Static)

        function node_handle = get_node_handle()
            persistent node_handle_
            if isempty(node_handle_)
                node_handle_ = ros2node("/tf_interface");
            end
            node_handle = node_handle_;
        end

        function tf_pub = get_tf_pub()
            persistent tf_pub_
            if isempty(tf_pub_)
                tf_pub_ = ros2publisher(tf_frame.get_node_handle(),'rdkdc/tf_msg','geometry_msgs/TransformStamped');
            end
            tf_pub = tf_pub_;
        end

        function tf_tree = get_tf_tree()
            persistent tf_tree_
            if isempty(tf_tree_)
                tf_tree_ = ros2tf(tf_frame.get_node_handle);
            end
            tf_tree = tf_tree_;
        end
    end
    
    methods
        
        % constructor
        function self = tf_frame(base_frame_name, frame_name, g)
            self.frame_name = frame_name;
            self.base_frame_name = base_frame_name;
            self.pose = g;
            %self.tftree = ros2tf(self.get_node_handle);
            self.move_frame(base_frame_name,g);
        end
        
        % move the frame by g relative to ref_frame
        function move_frame(self,ref_frame_name,g)
            msg = ros2message('geometry_msgs/TransformStamped');
            msg.child_frame_id = convertStringsToChars(self.frame_name);
            msg.header.frame_id = convertStringsToChars(ref_frame_name);
            msg.header.stamp = ros2time(tf_frame.get_node_handle,'now');
            
            % geometry transformation
            q = rotm2quat(g(1:3,1:3));
            t = g(1:3,4);
            msg.transform.translation.x = t(1);
            msg.transform.translation.y = t(2);
            msg.transform.translation.z = t(3);
            msg.transform.rotation.w = q(1);
            msg.transform.rotation.x = q(2);
            msg.transform.rotation.y = q(3);
            msg.transform.rotation.z = q(4);
            
            send(self.get_tf_pub,msg);
        end
        
        function g = read_frame(self,ref_frame_name,optional_timeout)
            if nargin < 3 % default: wait indefinitely for frames to be valid
                tran = getTransform(tf_frame.get_tf_tree, convertStringsToChars(ref_frame_name), convertStringsToChars(self.frame_name), Timeout=Inf);
            else % if optional_timeout is passed in as argument, use that for Timeout value
                tran = getTransform(tf_frame.get_tf_tree, convertStringsToChars(ref_frame_name), convertStringsToChars(self.frame_name), Timeout=optional_timeout);
            end
            t = [tran.transform.translation.x, tran.transform.translation.y, tran.transform.translation.z];
            R = quat2rotm([tran.transform.rotation.w, tran.transform.rotation.x, tran.transform.rotation.y, tran.transform.rotation.z]);
            g = [R t';0 0 0 1];
        end
            
        % delete the frame in RVIZ, can be recoverd by move_frame
        function disappear(self)
            msg = ros2message('geometry_msgs/TransformStamped');
            msg.header.frame_id = 'Delete';
            msg.child_frame_id = convertStringsToChars(self.frame_name);
            send(self.get_tf_pub,msg);
        end
    end
    
end