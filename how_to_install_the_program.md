# Как прошить робота

Для того, чтобы заменить прошивку робота CrowBOT BOLT вам нужно установить Arduino IDE.

1.	Перейдите на сайт：https://arduino.cc
2.	Нажмите  'SOFTWARE' 
3.	Выберите подходящую для вашего устройства версию и скачайте пакет установщика.
   ![1](https://github.com/user-attachments/assets/33312775-3ca4-4491-ba1a-62c2c4ba9427)

4.	Установите программу
   
5.	Скачайте файл или скопируйте текст программы [Bolt_grc_program.ino](https://github.com/MIR-LLC/CrowBot_program/blob/main/Bolt_grc_program.ino) и откройте/вставьте в IDE.
      
 ![2](https://github.com/user-attachments/assets/f710330b-5f63-4de0-ab1e-d5f6fa39423d)

6.	Подключите CrowBOT к USB и включите его.
 
<img width="644" alt="3" src="https://github.com/user-attachments/assets/a9ce9c02-f667-4ff6-aa0d-a42d0d987f8e" />


7.	Нажмите File ->Preferences ->Additional boards manager URLs->Add the following URLs:

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 
 ![4](https://github.com/user-attachments/assets/10d065c5-b950-4d0f-9eeb-f34bef9de88e)

![5](https://github.com/user-attachments/assets/5db0681c-96db-4a32-8cc3-d7c962c8d908)


8.	Установите пакет работы с esp32. 
Нажмите Tools ->Board ->Board Manager. 
Введите в поиск "esp", найдите "esp32 by Espressif Systems", **обязательно выберите версию 2.0.3**, и нажмите Install. С другими версиями пакета программа не заработает. Если они у вас уже были установлены - удалите и установите версию 2.0.3
 ![6](https://github.com/user-attachments/assets/469785b9-dd04-46ea-8a75-9101cb0095e6)


9. Нажмите "Tools"->"Board" и выберите "ESP32-WROOM-DA Module" из выпадающего меню;
    ![7](https://github.com/user-attachments/assets/52876ff7-7f58-460c-9bd5-6a09496b54da)

 
10.	Нажмите “Tools” -> “Port”, и выберите соответствующий порт;
    
12.	Установите необходимые библиотеки:
    
 ![8](https://github.com/user-attachments/assets/a397c8ab-87d7-4381-8a58-faa7cbd9a090)

![9](https://github.com/user-attachments/assets/df8bc38c-68e7-4a23-bae9-b924d8c7fbe7)

 ![10](https://github.com/user-attachments/assets/370e5a68-7699-41b5-93b6-158c849d5d90)


 

12.	Нажмите кнопку "Upload" для перепрошивки робота
  ![11](https://github.com/user-attachments/assets/60dc89e6-1cea-4502-b13b-4e94310fa899)
  
 
13.	При удачной прошивке появится сообщение
    
 <img width="494" alt="12" src="https://github.com/user-attachments/assets/fb3e8dcb-080e-4b8a-9017-ad302a657215" />


14.	Выключите робота и отключите его от компьютера. Он перепрошит и готов к использованию.
15.	Включите его, включите плату управления (подключите к USB или пауэрбанку), не забудьте включить блютус на самом роботе (переключать внизу платы робота).
