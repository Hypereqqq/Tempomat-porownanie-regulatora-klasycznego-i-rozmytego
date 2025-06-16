from simpful import *
import numpy as np

class FuzzyPIController:
    def __init__(self):
        self._build_system()
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_d_error = 0.0

    def _build_system(self):
        self.FS = FuzzySystem()

        E_range = [-0.5, 0.5]  # błąd prędkości (m/s)
        dE_range = [-5, 5]  # zmiana błędu (m/s)
        U_range = [-1, 1] # sygnał sterujący (normalizowany)

        E = LinguisticVariable([
            FuzzySet(function=Triangular_MF(a=-0.5, b=-0.5, c=-0.2), term="NB"),
            FuzzySet(function=Triangular_MF(a=-0.5, b=-0.2, c=0), term="NS"),
            FuzzySet(function=Triangular_MF(a=-0.2, b=0, c=0.2), term="ZE"),
            FuzzySet(function=Triangular_MF(a=0, b=0.2, c=0.5), term="PS"),
            FuzzySet(function=Triangular_MF(a=0.2, b=0.5, c=0.5), term="PB"),
        ], universe_of_discourse=E_range, concept="Error")

        dE = LinguisticVariable([
            FuzzySet(function=Triangular_MF(a=-5, b=-5, c=-2), term="NB"),
            FuzzySet(function=Triangular_MF(a=-5, b=-2, c=0), term="NS"),
            FuzzySet(function=Triangular_MF(a=-2, b=0, c=2), term="ZE"),
            FuzzySet(function=Triangular_MF(a=0, b=2, c=5), term="PS"),
            FuzzySet(function=Triangular_MF(a=2, b=5, c=5), term="PB"),
        ], universe_of_discourse=dE_range, concept="DeltaError")

        U = LinguisticVariable([
            FuzzySet(function=Triangular_MF(a=-1, b=-1, c=-0.5), term="NB"),
            FuzzySet(function=Triangular_MF(a=-1, b=-0.5, c=0), term="NS"),
            FuzzySet(function=Triangular_MF(a=-0.5, b=0, c=0.5), term="ZE"),
            FuzzySet(function=Triangular_MF(a=0, b=0.5, c=1), term="PS"),
            FuzzySet(function=Triangular_MF(a=0.5, b=1, c=1), term="PB"),
        ], universe_of_discourse=U_range, concept="Control")

        self.FS.add_linguistic_variable("Error", E)
        self.FS.add_linguistic_variable("DeltaError", dE)
        self.FS.add_linguistic_variable("Control", U)

        # tabela reguł:
        # 
        # |      | NB  | NS  | ZE  | PS  | PB  |
        # |------|-----|-----|-----|-----|-----|
        # | NB   | NB  | NB  | NS  | ZE  | ZE  |
        # | NS   | NB  | NS  | NS  | ZE  | PS  |
        # | ZE   | NS  | NS  | ZE  | PS  | PS  |
        # | PS   | ZE  | PS  | PS  | PB  | PB  |
        # | PB   | PS  | PS  | PB  | PB  | PB  |


        rules = [
        # Pierwszy wiersz - Error = NB
        "IF (Error IS NB) AND (DeltaError IS NB) THEN (Control IS NB)",
        "IF (Error IS NB) AND (DeltaError IS NS) THEN (Control IS NB)",
        "IF (Error IS NB) AND (DeltaError IS ZE) THEN (Control IS NB)",
        "IF (Error IS NB) AND (DeltaError IS PS) THEN (Control IS NS)",
        "IF (Error IS NB) AND (DeltaError IS PB) THEN (Control IS ZE)",
        
        # Drugi wiersz - Error = NS
        "IF (Error IS NS) AND (DeltaError IS NB) THEN (Control IS NB)",
        "IF (Error IS NS) AND (DeltaError IS NS) THEN (Control IS NB)",
        "IF (Error IS NS) AND (DeltaError IS ZE) THEN (Control IS NS)",
        "IF (Error IS NS) AND (DeltaError IS PS) THEN (Control IS ZE)",
        "IF (Error IS NS) AND (DeltaError IS PB) THEN (Control IS PS)",
        
        # Trzeci wiersz - Error = ZE
        "IF (Error IS ZE) AND (DeltaError IS NB) THEN (Control IS NB)",
        "IF (Error IS ZE) AND (DeltaError IS NS) THEN (Control IS NS)",
        "IF (Error IS ZE) AND (DeltaError IS ZE) THEN (Control IS ZE)",
        "IF (Error IS ZE) AND (DeltaError IS PS) THEN (Control IS PS)",
        "IF (Error IS ZE) AND (DeltaError IS PB) THEN (Control IS PB)",
        
        # Czwarty wiersz - Error = PS
        "IF (Error IS PS) AND (DeltaError IS NB) THEN (Control IS NS)",
        "IF (Error IS PS) AND (DeltaError IS NS) THEN (Control IS ZE)",
        "IF (Error IS PS) AND (DeltaError IS ZE) THEN (Control IS PS)",
        "IF (Error IS PS) AND (DeltaError IS PS) THEN (Control IS PB)",
        "IF (Error IS PS) AND (DeltaError IS PB) THEN (Control IS PB)",
        
        # Piąty wiersz - Error = PB
        "IF (Error IS PB) AND (DeltaError IS NB) THEN (Control IS ZE)",
        "IF (Error IS PB) AND (DeltaError IS NS) THEN (Control IS PS)",
        "IF (Error IS PB) AND (DeltaError IS ZE) THEN (Control IS PB)",
        "IF (Error IS PB) AND (DeltaError IS PS) THEN (Control IS PB)",
        "IF (Error IS PB) AND (DeltaError IS PB) THEN (Control IS PB)"
    ]

        self.FS.add_rules(rules)

    def compute(self, error, v_curr, t, dt=0.1):
        self.last_d_error = (error - self.prev_error)
        self.prev_error = error

        # Filtracja d_error
        d_error = np.clip(self.last_d_error, -5, 5)

        # Uaktualnienie składnika całkującego
        self.integral += error * dt

        self.FS.set_variable("Error", error)
        self.FS.set_variable("DeltaError", d_error)

        output = self.FS.inference()
        u = output["Control"]

        # Dodanie lekkiego wpływu całki 
        u += 0.1 * np.tanh(self.integral / 10.0)  # ograniczamy wpływ
        u = np.clip(u, -1, 1)

        #print(f"error={error:.2f}, d_error={d_error:.2f}, output={u:.2f}")
        return u
    


    def segment_control_colors(self, time, control_values):
        segments = []
        colors = ["rgba(0,255,0,0.1)" if u > 0.05 else "rgba(255,0,0,0.1)" if u < -0.05 else "rgba(0,0,255,0.1)" for u in control_values]
        start_idx = 0
        for i in range(1, len(time)):
            if colors[i] != colors[start_idx]:
                segments.append((time[start_idx], time[i], colors[start_idx]))
                start_idx = i
        segments.append((time[start_idx], time[-1], colors[start_idx]))
        return segments