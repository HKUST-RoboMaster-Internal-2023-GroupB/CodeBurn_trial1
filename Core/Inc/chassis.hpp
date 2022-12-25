void chassisResolve(int x, int y, int w, int& wheel1, int& wheel2, int& wheel3, int& wheel4){
    
    wheel1 = y + x + w;
    wheel2 = -y + x + w;
    wheel3 = y - x + w;
    wheel4 = -y - x + w;
}
