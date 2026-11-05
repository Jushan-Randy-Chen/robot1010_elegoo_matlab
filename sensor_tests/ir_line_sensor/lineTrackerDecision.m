function decision = lineTrackerDecision(onLine)
% Line-tracker steering rule for the three IR sensors [left middle right].
% onLine is 1 where a sensor sees black. Returns "forward", "left", "right"
% or "stop", the same rule as the course line-tracking example:
%   111 or 010 -> forward
%   100 or 110 -> left   (the line is under the left side)
%   001 or 011 -> right
%   000 or 101 -> stop   (line lost or split)
% Example: lineTrackerDecision([1 1 0])   % "left"

validateattributes(onLine, {'numeric', 'logical'}, {'numel', 3});
onLine = logical(onLine(:).');
left = onLine(1); middle = onLine(2); right = onLine(3);

if left && ~right
    decision = "left";
elseif right && ~left
    decision = "right";
elseif middle
    decision = "forward";
else
    decision = "stop";
end
end
