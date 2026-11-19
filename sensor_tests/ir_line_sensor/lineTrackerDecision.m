function decision = lineTrackerDecision(onLine)
% Line-tracker steering rule for the three IR sensors [left middle right].
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
