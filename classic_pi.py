class ClassicPIController:
    def __init__(self, Kp=1, Ti=1, output_limit=(-1, 1)):
        
        self.Kp = Kp
        self.Ti = Ti  # Okres całkowania zamiast Ki
        self.output_min, self.output_max = output_limit
        self.integral = 0.0

    def compute(self, error, v_curr, t, dt=0.01):
        
        # Dynamiczne obliczenie Ki na podstawie wzoru: Ki = Kp * dt / Ti
        Ki = self.Kp * dt / self.Ti if self.Ti != 0 else 0  # Zabezpieczenie przed dzieleniem przez zero
        
        self.integral += error * dt
        u = self.Kp * error + Ki * self.integral

        # Saturacja sygnału wyjściowego
        u = max(min(u, self.output_max), self.output_min)
        return u