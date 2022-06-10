addpath("lib\")
kb = HebiKeyboard();
while true
    state = read(kb);
    if all(state.keys('x'))
        disp("x and 0 are borth pressed")
    end
    disp("hi")
    pause(0.01)
end