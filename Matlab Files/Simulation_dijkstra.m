%Defining Function
function [len_path,reversed_nodes_prnt] = Simulation_dijkstra()

% Set up the workspace
workspace_size = 5;
grid_size = 1;
num_nodes = workspace_size^2;
nodes_per_row = workspace_size;
nodes = zeros(num_nodes, 2);

%Setting up the figure
figure;
grid on;
hold on;

% Set up the nodes
for i = 1:num_nodes
    row = ceil(i / nodes_per_row);       %Defines the row number
    col = i - (row-1) * nodes_per_row;   %Defines the column number
    x = (col - 0.5) * grid_size;         %Defines x coordinate of the node
    y = (row - 0.5) * grid_size;         %Defines y coordinate of the node
    
    %Defining Obstacle at particular Locations
    if col==1 && row==3
        plot(x, y, 'bo', 'MarkerSize', 10, 'LineWidth', 2,'Color','r'); % plot blue circle for continued point
        continue;
    end
    if col==4 && row==4
        plot(x, y, 'bo', 'MarkerSize', 10, 'LineWidth', 2,'Color','r'); % plot blue circle for continued point
        continue;
    end

    nodes(i, :) = [x, y];     %Stores the x and y coordinates of the node
end


% Set up the connections between nodes
connections = zeros(num_nodes, num_nodes); %Defining a matrix of 25x25 with all 0's as the element

for i = 1:num_nodes
    for j = i+1:num_nodes
        dist = norm(nodes(i,:) - nodes(j,:));
        %We have defined a threshold to select that value of distance
        if dist <= sqrt(2) * grid_size
            %Updating the connection matrix as distance between node 1 to
            %2=distance between node 2 to 1
            connections(i,j) = dist;
            connections(j,i) = dist;
        end
    end
end


% Set up the start and goal nodes
start_node = 1;
goal_node = num_nodes;

% Set up the Dijkstra's algorithm
distances = Inf(num_nodes, 1);
distances(start_node) = 0;
visited = zeros(num_nodes, 1);
prev = zeros(num_nodes, 1);

%Plotting the nodes
for i = 1:num_nodes
    plot(nodes(i,1), nodes(i,2), 'o', 'MarkerSize', 10, 'Color', 'k');
    %     nodes
end
plot(nodes(start_node,1), nodes(start_node,2), 'o', 'MarkerSize', 10, 'Color', 'g');%1st row contains the coordinates of start point
% nodes(start_node,1)
plot(nodes(goal_node,1), nodes(goal_node,2), 'o', 'MarkerSize', 10, 'Color', 'r');%25th row contains the coordinates of goal point
title('Dijkstra Algorithm Demonstration');
xlim([0, workspace_size]);
ylim([0, workspace_size]);
distance_min=[];
% Run the Dijkstra's algorithm and animate it
while ~visited(goal_node)

    % Find the unvisited node with the smallest distance
    unvisited_nodes = find(~visited); %find function returns the non zero value.So if it is non zero then node is visited.
    [distance_min, min_node] = min(distances(unvisited_nodes));%Minimum distance and index of that particular node. We are ignoring the minimum value 
                                                     %just keeping the index
    node = unvisited_nodes(min_node);

    % Mark the node as visited
    visited(node) = 1;

    % Update the distances of the neighboring nodes
    neighbors = find(connections(node,:));
    for neighbor = neighbors
        if ~visited(neighbor)
            dist = distances(node) + connections(node, neighbor);
            %we will update the index if distance of neighbour is smaller
            %than previous distance
            if dist < distances(neighbor)
                distances(neighbor) = dist;
                prev(neighbor) = node;
            end
        end
    end

    % Update the animation
    for i = 1:num_nodes
        if visited(i)
            plot(nodes(i,1), nodes(i,2), 'o', 'MarkerSize', 10, 'Color', 'm');
            %             nodes(i,1)
        end
    end
end
nodes_prnt=[];

% Plot the final path
path = goal_node;
while path ~= start_node
    plot(nodes(path,1), nodes(path,2), 'o', 'MarkerSize', 10,'Color', 'b');
    nodes_prnt=[nodes_prnt,nodes(path,1),nodes(path,2)];
    path = prev(path);
end
%Data pre processing to feed date to control algorithm
reversed_nodes_prnt = flip(nodes_prnt);
for i = 1:2:length(reversed_nodes_prnt)-1
    temp = reversed_nodes_prnt(i);
    reversed_nodes_prnt(i) = reversed_nodes_prnt(i+1);
    reversed_nodes_prnt(i+1) = temp;
    
end

len_path=length(reversed_nodes_prnt)/2;
hold off;


