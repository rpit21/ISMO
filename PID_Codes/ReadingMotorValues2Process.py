import serial
import time

# Configura el puerto serial y la velocidad de baudios (ajusta según tu configuración)
ser = serial.Serial('COM4', 9600)  # Reemplaza 'COM3' con el puerto serial correcto

# Abre el archivo de texto para escribir los datos
with open('motor_velocity_data.txt', 'w') as file:
    start_time = time.time()  # Obtiene el tiempo de inicio del script
    data_count = 0  # Contador de datos leídos
    
    file.write(f"TIEMPO W_MR W_ML (seg rad/s)\n")

    #Envio de velocidades angulares por serial a motores aquiridos en consola 
    ang_vels=input("Introduce la velocidad de las ruedas MR MI (separadas por un espacio 0-255): ")
    ser.write((ang_vels+"\n").encode('utf-8'))

    while data_count < 1500:  # Lee exactamente 500 datos
        if ser.in_waiting > 0:

            data = ser.readline().decode('utf-8',errors='ignore').strip() # lee el dato, transforma a string, y elimina cualquier espacio en blaco al inicio o final

            try: 
                w_mr, w_ml = map(float,data.split()) #divide la cadena 

                timestamp = time.time() - start_time  # Calcula el tiempo transcurrido en segundos

                print(f"tiempo:{timestamp} MR:{w_mr} ML:{w_ml}")
                file.write(f"{timestamp} {w_mr} {w_ml}\n")
                file.flush()  # Asegura que los datos se escriben en el archivo inmediatamente

                data_count += 1  # Incrementa el contador de datos
            except ValueError:
                print(f"Datos inválidos recibidos: {data}")
        #time.sleep(1)  # Espera 1 segundo entre lecturas
                
ser.write(("0 0\n").encode('utf-8')) # Envio de datos para apagar los motores finalizado el proceso de lectura
ser.close()  # Cierra el puerto serial al final
