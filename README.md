# Symulator tempomatu z regulatorami PI i rozmytym PI

## Opis projektu

Projekt jest interaktywną aplikacją symulującą działanie tempomatu w pojazdach. Umożliwia porównanie dwóch typów regulatorów:
- Klasycznego regulatora PI (Proporcjonalno-Całkującego)
- Rozmytego regulatora PI bazującego na logice rozmytej (fuzzy logic)

Aplikacja pozwala na symulację zachowania różnych typów pojazdów na trasie o zmiennym nachyleniu, z różnymi zadanymi prędkościami oraz przy różnych parametrach regulatorów.

## Struktura projektu

- `vehicle_model.py` - model pojazdu i funkcja symulacji
- `classic_pi.py` - implementacja klasycznego regulatora PI
- `fuzzy_pi.py` - implementacja regulatora rozmytego PI
- `app.py` - aplikacja Dash z interfejsem użytkownika

## Parametry pojazdów

Dostępne są cztery typy pojazdów z różnymi parametrami:

| Typ pojazdu | Masa [kg] | Powierzchnia czołowa [m²] | Współczynnik oporu | Maksymalna siła [N] |
|------------|-----------|--------------------------|-------------------|-------------------|
| osobowy    | 1500      | 2.2                      | 0.29              | 4500              |
| sportowy   | 1300      | 1.9                      | 0.27              | 7500              |
| van        | 2200      | 2.8                      | 0.33              | 5000              |
| ciężarówka | 15000     | 5.0                      | 0.6               | 40000             |

## Porównanie regulatorów

### Regulator klasyczny PI

Regulator PI działa na podstawie równania:
```
u(t) = Kp * [e(t) + (1/Ti) * ∫e(τ)dτ]
```
gdzie:
- `u(t)` - sygnał sterujący
- `e(t)` - błąd (różnica między prędkością zadaną a aktualną)
- `Kp` - wzmocnienie proporcjonalne
- `Ti` - stała czasowa całkowania

Charakteryzuje się:
- Deterministycznym, liniowym charakterem działania
- Prostotą implementacji i strojenia (dwa parametry)
- Dobrze znaną teorią i metodami strojenia

### Regulator rozmyty PI

Regulator rozmyty PI wykorzystuje logikę rozmytą i reguły decyzyjne:
1. Zamienia wartości liczbowe na zmienne lingwistyczne
2. Stosuje zestaw reguł "JEŻELI-TO" do określenia działania
3. Przekształca wynik rozmyty na konkretną wartość sterowania

Charakteryzuje się:
- Nieliniowym zachowaniem dostosowanym do różnych sytuacji
- Lepszą odpornością na zakłócenia i nieliniowości obiektu
- Bardziej "ludzkim" zachowaniem i łagodniejszymi przejściami między stanami
- Możliwością intuicyjnego projektowania bez szczegółowego modelu matematycznego

## Jak korzystać z aplikacji

1. Uruchom aplikację poleceniem: `python app.py`
2. W przeglądarce otwórz adres: `http://127.0.0.1:8050/`
3. Skonfiguruj parametry:
   - Wybierz typ pojazdu
   - Ustaw parametry regulatorów (Kp i Ti dla klasycznego PI)
   - Zdefiniuj profil trasy (czasy i nachylenia odcinków)
   - Określ prędkości zadane dla każdego odcinka
4. Kliknij przycisk "Symuluj"
5. Przeanalizuj wyniki na wykresach:
   - Porównanie prędkości dla obu regulatorów
   - Siły działające na pojazd
   - Sygnały sterujące

## Fizyka symulacji

Symulacja uwzględnia:
- Siłę napędową pojazdu (ograniczoną do maksymalnej wartości dla danego typu)
- Opór aerodynamiczny zależny od prędkości i parametrów pojazdu
- Siłę grawitacji zależną od nachylenia drogi
- Masę pojazdu wpływającą na jego dynamikę

## Wymagania

- Python 3.7+
- Dash
- Plotly
- NumPy
- scikit-fuzzy (dla regulatora rozmytego)

## Autorzy

Ten projekt został stworzony jako część zajęć akademickich na temat inteligentych systemów sterowania.
