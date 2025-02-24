/*
 * extractValue(value)
 * showModal(modalId)
 * getTimeWindows()
 * close_time_alert()
 * show_2st_alert()
 * show_1st_alert()
 * show_3st_alert()
 * give_time()
 * confirmDate()
 * saveDateTime()
 * validateData(ip, mask, gateway)
 * saveChanges()
 * closeCustomModal()
 * confirmChanges()
 * invaliddata()
 * closemodal()
 * saveData()
 * saveDataAdmin()
 * setrelay()
 * test()
 *
 * parseCurrent(str)
 * parseRelayButton(str)
 * parseALERT(str)
 *
 * decodeLogEntry(bytes)
 * appendLogEntry(log)
 * renderPage(page)
 * generatePageNumbers(currentPage, totalPages)
 * updateSaveButtonState()
 * renderFilterMenu()
 * parseLogData(byteArray)
 * fetchLogs()
 *
 * validateDataAdmin(macFull, serial)
 * saveChangesAdmin()
 * confirmChangesAdmin()
 *
 * redirectToUpdatePage()
 * editbuttoninline()
 * changeButtonTextBasedOnResolution()
 *
 * showAlert(alertType)
 * closeAlertModal()
 *
 * go_to_log()
 */

let initialData = {
    mac: '',
    ip: '',
    mask: '',
    gateway: '',
	pin: ''
};

let timeAlertShown = false;
var globalHours = "";
var globalMinutes = "";
var globalSeconds = "";
let selectedDay = null;
let canvas_logic = 1;

function extractValue(value) {
	return value.replace(/<!--.*?-->/, '').trim(); // Убираем шаблон и лишние пробелы
}

/*Функции для запроса времени (календарик, подтяжка с винды и прочее) на index.shtml*/

function showModal(modalId) {
    // Скрываем все окна с классом .modal_time
    document.querySelectorAll('.modal_time').forEach(function(modal) {
        modal.style.display = 'none';
    });
    // Показываем окно с нужным id
    document.getElementById(modalId).style.display = 'block';
}

function getTimeWindows() {
    const now = new Date();

    let day = now.getDate();
    let month = now.getMonth() + 1; // месяцы начинаются с 0
    let year = now.getFullYear();
    let hours = now.getHours();
    let minutes = now.getMinutes();
    let seconds = now.getSeconds();

    // Дополняем нулём, если нужно
    day = day < 10 ? '0' + day : day;
    month = month < 10 ? '0' + month : month;
    hours = hours < 10 ? '0' + hours : hours;
    minutes = minutes < 10 ? '0' + minutes : minutes;
    seconds = seconds < 10 ? '0' + seconds : seconds;

    const date = `${day}-${month}-${year}`;
    const time = `${hours}-${minutes}-${seconds}`;

    console.log('Синхронизировано: ' + date + ' время: ' + time);

    const dateTimeParam = `${date}-${time}`; // формируем строку, как "31-12-2023-10-20-59"
    
    fetch(`/save?date=${dateTimeParam}`, {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        }
    });
}


 function close_time_alert(){
	var timeSection1 = document.getElementById('time1');
	if(timeSection1)
	{
		timeSection1.style.display = 'none';
	}
	
	var timeSection3 = document.getElementById('time3');
	if(timeSection3)
	{
		timeSection3.style.display = 'none';	
	}
 }

 function show_2st_alert(){
      showModal("time2")
    }

 function show_1st_alert(){
      showModal("time1")
    }

 function show_3st_alert(){
 
	globalHours = document.getElementById('hours').value.trim();
	globalMinutes = document.getElementById('minutes').value.trim();
	globalSeconds = document.getElementById('seconds').value.trim();
	    
	console.log("globalHours:", globalHours);
	console.log("globalMinutes:", globalMinutes);
	console.log("globalSeconds:", globalSeconds);
	  
	if (!globalHours || !globalMinutes || !globalSeconds) {
	alert('Пожалуйста, заполните все поля');
	return; 
	} 
     showModal("time3")
    }



function give_time(){
	    fetch(`/save?givetime= 0`, {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        }
    });
}


document.addEventListener('DOMContentLoaded', function() {
  const maxValues = {
	hours: 24,
	minutes: 59,
	seconds: 59
  };

  const fields = ['hours', 'minutes', 'seconds'];

  fields.forEach((field, index) => {
	const input = document.getElementById(field);
	if(input) {
	  input.addEventListener('input', function(e) {
		this.value = this.value.replace(/\D/g, '');
		let value = parseInt(this.value, 10);
		if (!isNaN(value) && value > maxValues[field]) {
		  this.value = maxValues[field];
		}
		if (this.value.length === this.maxLength) {
		  const nextField = fields[index + 1];
		  if (nextField) {
			document.getElementById(nextField).focus();
		  }
		}
	  });

	  input.addEventListener('keydown', function(e) {
		if (e.key === 'Backspace' && this.value.length === 0) {
		  const prevField = fields[index - 1];
		  if (prevField) {
			document.getElementById(prevField).focus();
		  }
		}
	  });
	}
  });
  
  // Обработка клика по дням календаря
  document.querySelectorAll('.calendar-days .day').forEach(day => {
	day.addEventListener('click', function() {
	  document.querySelectorAll('.calendar-days .day').forEach(d => d.classList.remove('selected'));
	  this.classList.add('selected');
	  selectedDay = this.getAttribute('data-day');
	});
  });
});

   

function confirmDate() {
  let month = document.getElementById('calendar-month').value;
  month = month.padStart(2, '0');
  const year = document.getElementById('calendar-year').value;
  if (!selectedDay) {
	alert('День не выбран');
	return;
  }
  
  selectedDay = selectedDay.padStart(2, '0');
  
  console.log('Выбрана дата: ' + selectedDay + '-' + month + '-' + year + '- время: ' + globalHours + '-' + globalMinutes + '-' + globalSeconds);
  saveDateTime();

}
	
	

function saveDateTime() {
    // Собираем данные даты и времени
    let month = document.getElementById('calendar-month').value;
	month = month.padStart(2, '0');
    const year = document.getElementById('calendar-year').value;
    const date = `${selectedDay}-${month}-${year}`;
    let hoursString = globalHours.padStart(2, '0');
	let minutesString = globalMinutes.padStart(2, '0');
	let secondsString = globalSeconds.padStart(2, '0');

	const time = `${hoursString}-${minutesString}-${secondsString}`;

    // Логируем для проверки
    console.log('Выбрана дата: ' + date + ' время: ' + time);

    // Формируем единую строку
    const dateTimeParam = `${date}-${time}`; // "31-12-2023-10-20-59"

    // Отправляем данные на сервер
    fetch(`/save?date=${dateTimeParam}`, {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        }
    });
}






/*Основная функция, общая для многих страниц*/
window.onload = function() {
	console.log('функции загрузились');
    var inputField1 = document.getElementById("field3");
    if (inputField1) {
        inputField1.value = inputField1.value.replace("<!--#MASK-->", "");
    }
	
	var inputField4 = document.getElementById("field1");
    if (inputField4) {
        inputField4.value = inputField4.value.replace("<!--#MAC-->", "");
    }
	
	var inputField2 = document.getElementById("field2");
    if (inputField2) {
        inputField2.value = inputField2.value.replace("<!--#IP-->", "");
    }
	
	var inputField3 = document.getElementById("field4");
    if (inputField3) {
        inputField3.value = inputField3.value.replace("<!--#GETAWEY-->", "");
    }
	var inputField6 = document.getElementById("field6");
    if (inputField6) {
        inputField6.value = inputField6.value.replace("<!--#SERIAL-->", "");
    }
	
	var inputField8 = document.getElementById("field8");
    if (inputField8) {
        inputField8.value = inputField8.value.replace(/<!--#\w+-->/g, "");
    }
	
		var inputField21 = document.getElementById("field20");
    if (inputField21) {
        inputField21.value = inputField21.value.replace("<!--#SPEED-->", "");
    }
		var inputField22 = document.getElementById("field21");
    if (inputField22) {
        inputField22.value = inputField22.value.replace("<!--#PARITY-->", "");
    }
		var inputField23 = document.getElementById("field22");
    if (inputField23) {
        inputField23.value = inputField23.value.replace("<!--#STOPB-->", "");
    }
	
	var inputField9 = document.getElementById("field9");
    if (inputField9) {
        inputField9.value = inputField9.value.replace("<!--#SERIAL-->", "");
    }
	
	var inputField10 = document.getElementById("field10");
    if (inputField10) {
        inputField10.value = inputField10.value.replace("<!--#SOFT-->", "");
    }
	
	var inputField11 = document.getElementById("field11");
    if (inputField11) {
        inputField11.value = inputField11.value.replace(/<!--#\w+-->/g, "");
    }
	
	var inputFieldAlert = document.getElementById("field31");
    if (inputFieldAlert) {
        inputFieldAlert.value = inputFieldAlert.value.replace("<!--#ALERT-->", "");
    }
	
	
	if (inputField2) {
		initialData.mask = document.getElementById('field3').value;
		initialData.ip = document.getElementById('field2').value;
		initialData.gateway = document.getElementById('field4').value;
		var inputField99 = document.getElementById("field20");
		 console.log('взяли filed20');
	}
	
	/*заход в этот блок означает, что мы на index.shtml, весь код внутри IF относится к данной странице*/
	if(inputField99)
	{
		 console.log('зашли в блок с графиком');
		// Устанавливаем значение скорости RS-485
		const defaultSpeed = extractValue(document.getElementById('field20').value);
		const speedSelect = document.getElementById('field4-speed');
		speedSelect.value = defaultSpeed; // Устанавливаем значение в select

		// Устанавливаем значение стоп-бита
		const defaultStopBit = document.getElementById('field22').value;
		const stopbitSelect = document.getElementById('field4-stopbit');
		stopbitSelect.value = defaultStopBit; // Устанавливаем значение в select

		// Устанавливаем значение четности
		const defaultParity = document.getElementById('field21').value;
		const paritySelect = document.getElementById('field4-parity');
		paritySelect.value = defaultParity; // Устанавливаем значение в select
		
		
		initialData.rs485speed = defaultSpeed;
		initialData.rs485stopbit = defaultStopBit;
		initialData.rs485parity = defaultParity;;
		
		
		console.log("Speed value:", speedSelect.value);
		console.log("Stop Bit value:", document.getElementById('field22').value);
		console.log("Parity value:", document.getElementById('field21').value);
		
		
		//настройка размеров кнопок
		const relayButton = document.querySelector('.button-relay');
		const testButton = document.querySelector('.button-test');

		// Подхватить размеры первой кнопки
		const { width, height } = getComputedStyle(relayButton);
		testButton.style.width = width;
		testButton.style.height = height;
		
		
		const dataPoints = [];

		  function addDataPoint(newValue) {
			dataPoints.push(newValue);
			if (dataPoints.length > 25) {
			  dataPoints.shift();
			}
		  }

		  function drawChart() {
			const canvas = document.getElementById('chartCanvas');
			if(canvas_logic == "1")
			{
			const ctx = canvas.getContext('2d');

			// Очищаем холст
			ctx.clearRect(0, 0, canvas.width, canvas.height);

			// Если мало данных, ничего не рисуем
			if (dataPoints.length < 2) return;

			// Параметры
			const padding = 30;
			const w = canvas.width - 2 * padding;
			const h = canvas.height - 2 * padding;

			// Парсим порог из DOM
			const thresholdSpan = document.getElementById('threshold-value');
			// Если там в тексте "5 mA" или что-то такое, режем лишнее
			let thresholdVal = parseFloat(thresholdSpan.textContent.replace(/[^\d.-]/g, ''));

			if (isNaN(thresholdVal)) {
			  thresholdVal = 0;
			}

			// Динамический максимум (берём порог и докидываем к нему 20-30%)
			// Но если данные выше этой планки, пусть масштабируется под них
			const rawMin = Math.min(...dataPoints);
			const rawMax = Math.max(...dataPoints);
			
			// Минимум можно уронить в 0 (если нет смысла показывать отрицательные),
			// либо берём фактический минимум данных (на случай, если вдруг меньше нуля).
			const minVal = Math.min(0, rawMin);

			let suggestedMax = thresholdVal * 1.3; 
			let maxVal = Math.max(suggestedMax, rawMax); 

			// Если вдруг всё по нулям, чтоб не делить на 0
			const range = (maxVal - minVal) || 1;

			// Функция для преобразования данных -> координаты Y на холсте
			function getY(val) {
			  // Чем больше val, тем выше точка, но на canvas ось Y вниз
			  return padding + h - ((val - minVal) / range) * h;
			}

			// Рисуем сетку по Y (шаг 1 мА)
			// Чтобы примерно влезть, берём целую часть от minVal до maxVal
			ctx.strokeStyle = '#cccccc';
			ctx.lineWidth = 1;
			ctx.beginPath();
			
			  const yStep = maxVal / 10; // Шаг сетки по Y

			  for (let yGrid = 0; yGrid <= maxVal; yGrid += yStep) {
				const gy = getY(yGrid);
				ctx.moveTo(padding, gy);
				ctx.lineTo(padding + w, gy);
				ctx.font = '10px sans-serif';
				ctx.fillStyle = '#444';
				ctx.fillText(yGrid.toFixed(1) + 'mA', 0, gy + 3);
			  }
			  ctx.stroke();
			
			// Рисуем сетку по X (шаг 1 сек, но у нас данные приходят каждые 0.5с)
			// Считаем, что индекс массива dataPoints – это n-я точка, значит время = 0.5*n
			ctx.beginPath();
			for (let i = 0; i < dataPoints.length; i++) {
			  const timeSec = i * 0.5; // каждая точка = 0.5 сек
			  // Рисуем вертикальную линию, если timeSec целое
			  if (timeSec % 1 === 0) {
				const x = padding + (i * (w / (dataPoints.length - 1)));
				ctx.moveTo(x, padding);
				ctx.lineTo(x, padding + h);
				// Подпись внизу
				ctx.font = '10px sans-serif';
				ctx.fillStyle = '#444';
				ctx.fillText(timeSec + 's', x - 5, padding + h + 10);
			  }
			}
			ctx.stroke();

			// Теперь сама линия графика
			ctx.beginPath();
			dataPoints.forEach((value, index) => {
			  const x = padding + index * (w / (dataPoints.length - 1));
			  const y = getY(value);
			  if (index === 0) {
				ctx.moveTo(x, y);
			  } else {
				ctx.lineTo(x, y);
			  }
			});

			ctx.strokeStyle = 'blue';
			ctx.lineWidth = 2;
			ctx.stroke();

			// Рисуем красную линию порога
			const thresholdY = getY(thresholdVal);
			ctx.beginPath();
			ctx.moveTo(padding, thresholdY);
			ctx.lineTo(padding + w, thresholdY);
			ctx.strokeStyle = 'red';
			ctx.lineWidth = 1.5;
			ctx.stroke();
			}
		  }
		  
		  
		
		
		
			let alert1 = 0;
			let alert2 = 0;

			let alertbefore = 0;

			function updateValues() {
			fetch('/json.shtml')
			  .then(response => response.text())
			  .then(fullText => {
				const currentVal = parseCurrent(fullText);
				addDataPoint(Number(currentVal));
				drawChart();
				document.getElementById('current-value').textContent = currentVal;
				
				 const relayStatus = parseRelayButton(fullText);
				 const someAlert_temp = parseALERT(fullText);
				 let someAlert = 0;
				 
				 if ((someAlert_temp & 0x01) === 0x01) 
				 {
					 someAlert = 1;
				 }
				 
				 if ((someAlert_temp & 0x02) === 0x02)
				 {
					 someAlert = 2;
				 }
				 
				if ((someAlert_temp & 0x04) === 0x04) {
					 if (!timeAlertShown) {
                    console.log("Есть запрос на время");
                    timeAlertShown = true;
                    show_1st_alert();
                }
				} else {
					// Флаг сброшен
					close_time_alert();
					timeAlertShown = false;
				}
				if(alertbefore != someAlert)
				  {
					if(someAlert == 1)
					  {
						alert1 = 0;
					  }
					else if(someAlert == 2)
					  {
						alert2 = 0;
					  }
				  }	
				if(alert1 == 0)
				{
				 if(someAlert == 1)
				 {
					 alert1 = 1
					 setTimeout(function() 
						{
							showAlert(1);
						}, 2000);
				 }
				}
				if(alert2 == 0)
				{
				 if(someAlert == 2)
				 {
					 alert2 = 1;
					 setTimeout(function() 
						{
							showAlert(2);
						}, 2000);
				 }
				}
				alertbefore = someAlert; 
				const relayButton = document.getElementById('buttonrel');
				if (relayButton) {
					relayButton.innerHTML = relayStatus;
				} else {
					console.error("Кнопка с id 'buttonrel' не найдена.");
				}
			  })
			  .catch(error => {
				console.error('Ошибка парсера', error);
			  });
			}

		  function parseCurrent(str) {		  
			console.log("Полный ответ сервера:", str); // Выводим полный текст ответа  
			const regex = /"current"\s*:\s*(\d+(?:\.\d+)?)/;
			const match = regex.exec(str);

			if (match) {
				console.log("Найдено значение current:", match[1]); // Если нашли число
				return match[1];
			  } else {
				console.error("Не удалось найти current в строке."); // Если ничего не нашли
				return 'N/A';
			  }
			return match && match[1] ? match[1] : 'N/A';
		  }
		  
		  
			function parseRelayButton(str) {
				// Регулярное выражение для поиска строки после <!--#RELAY--> до символа '/'
				const regex = /<!--#RELAY-->\s*([^/]+)/;
				const match = regex.exec(str);
				
				if (match) {
					const relayStatus = match[1].trim(); // Убираем лишние пробелы
					console.log("Найдено значение Relay:", relayStatus);
					return relayStatus;
				} else {
					console.error("Не удалось найти Relay в строке.");
					return 'N/A';
				}
			}
			
			
		  function parseALERT(str) {	  
			console.log("Полный ALERT", str); // Выводим полный текст ответа  
			const regex = /<!--#ALERT-->\s*(\d+)/;
			const match = regex.exec(str);
			if (match) {
				console.log("Найдено значение alert:", match[1]); // Если нашли число
				return match[1];
			  } else {
				console.error("Не удалось найти alert в строке."); // Если ничего не нашли
				return 'N/A';
			  }
			return match && match[1] ? match[1] : 'N/A';
		  }
			
		 changeButtonTextBasedOnResolution();

		// Установка интервала обновления каждые 500 мс
		setInterval(updateValues, 500);
		
	}

	/*логика для notion.shtml*/
	if (window.location.href.includes("notion.shtml")) {
		var errorPin = document.getElementById('field5').value;
	}
	
	if(errorPin)
	{
	errorPin = errorPin.replace("<!--#PIN-->", "").trim();
	if (errorPin.trim() == "0") 
	{
		document.getElementById("main-content").style.display = "none"; // Скрыть основную страницу
		document.getElementById("error-page").style.display = "block";  // Показать страницу с ошибкой

		setTimeout(function() 
		{
			window.location.href = "/index.shtml"; // Перенаправление 
		}, 5000);
	}
	setTimeout(function() 
		{
			window.location.href = `http://${initialData.ip}`; // Перенаправление 
		}, 35000);
	}
	


/*Логика для страницы с логами - log.shtml*/
var sortButton = document.getElementById("field213");
    if (sortButton) {
      console.log('функция 213');

      // Флаги и параметры
      let isLoading = false;
      let allLogsLoaded = false;
      let page = 1;

      // Переменные для пагинации и сортировки
      let allLogs = [];
      let currentPage = 1;
      const logsPerPage = 30;
      let sortAscending = false;

      // Обработчик клика по кнопке сортировки
      sortButton.addEventListener('click', function() {
        sortAscending = !sortAscending;
        currentPage = 1;
        renderPage(currentPage);
        updatePaginationControls();
      });
      
      let activeFilters = {};

      // Обновление активных фильтров при получении новых логов
      function updateActiveFilters(newLogs) {
        newLogs.forEach(log => {
          if (!(log.event in activeFilters)) {
            activeFilters[log.event] = true;
          }
        });
        // Перерисовываем меню фильтров
        renderFilterMenu();
      }

      // Получение отсортированного и отфильтрованного списка логов
      function getFilteredLogs() {
        let filteredLogs = allLogs.filter(log => activeFilters[log.event] !== false);
        filteredLogs.sort((a, b) => sortAscending ? b.number - a.number : a.number - b.number);
        return filteredLogs;
      }

      // Декодирование одной записи из 12 байт
      function decodeLogEntry(bytes) {
        if (bytes.length < 12) return null;
        const [sec, min, hour, day, month, year] = bytes.slice(0, 6);
        const formattedDate = `${String(hour).padStart(2, '0')}:${String(min).padStart(2, '0')}:${String(sec).padStart(2, '0')} ${String(day).padStart(2, '0')}.${String(month).padStart(2, '0')}.${2000 + year}`;

        const eventCode = bytes[6];
        const data = bytes.slice(7, 12);
        let eventText = "";
        let description = "";

        switch (eventCode) {
          case 0x30:
            eventText = "Изменение тока утечки";
            description = "Изменение тока утечки на 10%";
            break;
          case 0x31:
            eventText = "ТЕСТ";
            description = "Тестовое событие";
            break;
          case 0x32:
            eventText = "Предупреждение";
            description = `Системное предупреждение о превышении порога (${parseInt(data[0])} mA)`;
            break;
          case 0x33:
            eventText = "Защита";
            description = `Сработала защита (${parseInt(data[0])} mA)`;
            break;
          case 0x02:
            eventText = "IP адрес";
            description = data.slice(0, 4).join('.');
            break;
          case 0x03:
            eventText = "Маска подсети";
            description = data.slice(0, 4).join('.');
            break;
          case 0x04:
            eventText = "Шлюз";
            description = data.slice(0, 4).join('.');
            break;
          case 0x05:
            eventText = "Изменение состояния реле";
            description = (data[0] === 0x00 ? "Выключено" : "Включено");
            break;
          case 0x07:
            eventText = "Приняли новое ПО";
            description = "Обновление прошивки";
            break;
          case 0x09:
            eventText = "USART скорость";
            description = data.slice(0, 4).map(b => b.toString()).join('');
            break;
          case 0x10:
            eventText = "USART четность";
            description = String.fromCharCode(data[0]) === 'O' ? "odd" : "even";
            break;
          case 0x11:
            eventText = "USART стоп бит";
            description = data[0].toString();
            break;
          case 0x12:
            eventText = "C_phase_A";
            description = parseFloat(new DataView(new Uint8Array(data.slice(0, 4)).buffer).getFloat32(0, true)).toFixed(2);
            break;
          case 0x13:
            eventText = "C_phase_B";
            description = parseFloat(new DataView(new Uint8Array(data.slice(0, 4)).buffer).getFloat32(0, true)).toFixed(2);
            break;
          case 0x14:
            eventText = "C_phase_C";
            description = parseFloat(new DataView(new Uint8Array(data.slice(0, 4)).buffer).getFloat32(0, true)).toFixed(2);
            break;
          case 0x15:
            eventText = "R_leak_A";
            description = parseFloat(new DataView(data.buffer, data.byteOffset, 4).getFloat32(0, true)).toFixed(2);
            break;
          case 0x16:
            eventText = "R_leak_B";
            description = parseFloat(new DataView(data.buffer, data.byteOffset, 4).getFloat32(0, true)).toFixed(2);
            break;
          case 0x17:
            eventText = "R_leak_C";
            description = parseFloat(new DataView(data.buffer, data.byteOffset, 4).getFloat32(0, true)).toFixed(2);
            break;
          case 0x18:
            eventText = "TARGET_VALUE";
            description = data[0].toString();
            break;
          case 0x19:
            eventText = "WARNING_VALUE";
            description = data[0].toString();
            break;
          case 0x20:
            eventText = "Аппаратная защита";
            description = (data[0] === 0x01 ? "Включена" : "Выключена");
            break;
          default:
            eventText = "Неизвестное событие";
            description = `Код: 0x${eventCode.toString(16)}`;
            break;
        }
        return {
          date: formattedDate,
          event: eventText,
          description: description,
          timestamp: new Date(2000 + year, month - 1, day, hour, min, sec).getTime()
        };
      }

      // Добавление записи в DOM
      function appendLogEntry(log) {
		  const container = document.getElementById('logsContainer');
		  const item = document.createElement('div');
		  item.className = 'log-item';
		  item.innerHTML = `
			<div class="log-number">${log.number}</div>
			<div class="log-date">${log.date}</div>
			<div class="log-event">${log.event}</div>
			<div class="log-description">${log.description}</div>
		  `;
		  container.appendChild(item);
		}

      // Рендеринг страницы логов
      function renderPage(page) {
        const container = document.getElementById('logsContainer');
        container.innerHTML = '';
        const filteredLogs = getFilteredLogs();
        const startIndex = (page - 1) * logsPerPage;
        const pageLogs = filteredLogs.slice(startIndex, startIndex + logsPerPage);
        pageLogs.forEach(log => appendLogEntry(log));
      }

      // Генерация номеров страниц
      function generatePageNumbers(currentPage, totalPages) {
        const pages = [];
        if (totalPages <= 7) {
          for (let i = 1; i <= totalPages; i++) pages.push(i);
          return pages;
        }
        pages.push(1);
        let startWindow = currentPage - 2;
        let endWindow = currentPage + 2;
        if (startWindow < 2) {
          startWindow = 2;
          endWindow = startWindow + 4;
        }
        if (endWindow > totalPages - 1) {
          endWindow = totalPages - 1;
          startWindow = endWindow - 4;
        }
        if (startWindow > 2) pages.push('...');
        for (let i = startWindow; i <= endWindow; i++) pages.push(i);
        if (endWindow < totalPages - 1) pages.push('...');
        pages.push(totalPages);
        return pages;
      }
	  
	  //кнопка скачать (активно/нет)
	  function updateSaveButtonState() {
		  const saveButton = document.getElementById('saveButton');
		  saveButton.disabled = !allLogsLoaded;
		}
		
		var saveButton = document.getElementById("saveButton");
		
		if (saveButton) {
		  saveButton.addEventListener('click', function() {
			// Формируем содержимое файла, например, по одному логу в строке:
			let content = allLogs.map(log => {
			  return `#${log.number} ${log.date} - ${log.event}: ${log.description}`;
			}).join('\n');
			
			// Создаем Blob с типом text/plain
			const blob = new Blob([content], { type: 'text/plain' });
			// Генерируем временный URL для Blob
			const url = window.URL.createObjectURL(blob);
			
			// Создаем временную ссылку и кликаем по ней для скачивания файла
			const a = document.createElement('a');
			a.href = url;
			a.download = 'logs_mszr_380.txt';
			document.body.appendChild(a);
			a.click();
			
			// Очищаем за собой
			document.body.removeChild(a);
			window.URL.revokeObjectURL(url);
		  });
		}
	  

      // Обновление пагинации
      function updatePaginationControls() {
		  const paginationContainer = document.getElementById('paginationControls');
		  const filteredLogs = getFilteredLogs();
		  const totalPages = Math.ceil(filteredLogs.length / logsPerPage) || 1;
		  const pages = generatePageNumbers(currentPage, totalPages);
		  let html = '<div class="pagination">';
		  pages.forEach(pageItem => {
			if (pageItem === '...') {
			  html += `<span class="page ellipsis">${pageItem}</span> `;
			} else {
			  html += `<span class="page ${pageItem === currentPage ? 'active' : ''}" data-page="${pageItem}">${pageItem}</span> `;
			}
		  });

		  // Если не все логи загружены, добавляем индикатор загрузки
		  if (!allLogsLoaded) {
			// Вариант 1: простой текст
			html += `<span class="loading">Загрузка...</span>`;
			
			// Вариант 2: анимированный индикатор (раскомментируйте нижеследующую строку и добавьте CSS)
			//html += `<span class="spinner"></span>`;
		  }

		  html += '</div>';
		  paginationContainer.innerHTML = html;
		  document.querySelectorAll('.page').forEach(el => {
			if (el.classList.contains('ellipsis')) return;
			el.addEventListener('click', function() {
			  currentPage = parseInt(this.getAttribute('data-page'));
			  renderPage(currentPage);
			  updatePaginationControls();
			});
		  });
		}

      // Отрисовка выпадающего меню фильтров
      function renderFilterMenu() {
        let menu = document.getElementById('filterMenu');
        menu.innerHTML = '<strong>Фильтр по событиям</strong><br>';
        for (let event in activeFilters) {
          let checkbox = document.createElement('input');
          checkbox.type = 'checkbox';
          checkbox.id = 'filter_' + event;
          checkbox.checked = activeFilters[event];
          checkbox.addEventListener('change', function() {
            activeFilters[event] = this.checked;
            renderPage(currentPage);
            updatePaginationControls();
          });
          let label = document.createElement('label');
          label.htmlFor = 'filter_' + event;
          label.style.marginLeft = '5px';
          label.textContent = event;
          let container = document.createElement('div');
          container.style.margin = '5px 0';
          container.appendChild(checkbox);
          container.appendChild(label);
          menu.appendChild(container);
        }
      }

      // Парсинг логов из Uint8Array
	  function parseLogData(byteArray) {
	  // Инициализация статического счётчика, если он ещё не определён
	  if (typeof parseLogData.logCounter === 'undefined') {
		parseLogData.logCounter = 0;
	  }

	  const startMarker = new Uint8Array([0x3c, 0x21, 0x2d, 0x2d, 0x23, 0x4c, 0x4f, 0x47, 0x2d, 0x2d, 0x3e]);
	  const endMarker = new Uint8Array([0x2f, 0x2f, 0x2f]);

	  function indexOfSubarray(array, subarray, start = 0) {
		for (let i = start; i <= array.length - subarray.length; i++) {
		  let found = true;
		  for (let j = 0; j < subarray.length; j++) {
			if (array[i + j] !== subarray[j]) {
			  found = false;
			  break;
			}
		  }
		  if (found) return i;
		}
		return -1;
	  }

	  const startIndex = indexOfSubarray(byteArray, startMarker, 0);
	  if (startIndex === -1) return [];
	  const endIndex = indexOfSubarray(byteArray, endMarker, startIndex + startMarker.length);
	  if (endIndex === -1) return [];
	  const logsBytes = byteArray.slice(startIndex + startMarker.length, endIndex);

	  const terminationPattern = new Uint8Array([13, 10, 45, 13, 10]);
	  if (indexOfSubarray(logsBytes, terminationPattern) !== -1) {
		allLogsLoaded = true;
		updatePaginationControls();
	  }
		
		updateSaveButtonState();
		
	  const logs = [];
	  for (let i = 0; i + 12 <= logsBytes.length; i += 12) {
		const entryBytes = logsBytes.slice(i, i + 12);
		if (entryBytes.every(byte => byte === 0)) continue;
		if (entryBytes.every(byte => byte === 0xFF)) continue;
		if (
		  entryBytes[0] === 13 &&
		  entryBytes[1] === 10 &&
		  entryBytes[2] === 45 &&
		  entryBytes[3] === 13 &&
		  entryBytes[4] === 10
		) continue;
		const logEntry = decodeLogEntry(entryBytes);
		if (logEntry) logs.push(logEntry);
	  }


	  // Присваиваем уникальную нумерацию, продолжая с предыдущего значения
	  logs.forEach((log) => {
		parseLogData.logCounter++;
		log.number = parseLogData.logCounter;
	  });

	  updateActiveFilters(logs);
	  return logs;
	}

      // Загрузка логов с сервера
      async function fetchLogs() {
		  if (allLogsLoaded) return;

		  try {
			// Отправляем запрос к /logdata.shtml и ждём ответа
			const response = await fetch('/logdata.shtml');
			if (!response.ok) {
			  throw new Error('Ошибка сети');
			}
			const buffer = await response.arrayBuffer();
			const bytes = new Uint8Array(buffer);
			const newLogs = parseLogData(bytes);
			
			if (newLogs.length) {
			  allLogs = allLogs.concat(newLogs);
			  updatePaginationControls();
			  updateActiveFilters(newLogs);
			  renderPage(currentPage);
			}
		  } catch (err) {
			console.error("Ошибка получения логов:", err);
		  } finally {
			// Как только получен ответ и обработан, сразу запускаем следующий запрос,
			// если логи ещё не закончились
			if (!allLogsLoaded) {
			  fetchLogs();
			}
		  }
		}

      
      // Обработчик кнопки фильтров
      var filtersButton = document.getElementById("filtersButton");
      if (filtersButton) {
        filtersButton.addEventListener('click', function(e) {
          let menu = document.getElementById('filterMenu');
          if (menu.style.display === 'block') {
            menu.style.display = 'none';
          } else {
            menu.style.display = 'block';
            let rect = filtersButton.getBoundingClientRect();
            let wrapperRect = document.querySelector('.logs-wrapper').getBoundingClientRect();
            menu.style.top = (rect.bottom - wrapperRect.top) + 'px';
            menu.style.left = (rect.left - wrapperRect.left) + 'px';
          }
          e.stopPropagation();
        });
      }
      
      // Скрытие меню при клике вне его области
      document.addEventListener('click', function(e) {
        let menu = document.getElementById('filterMenu');
        if (menu && e.target !== menu && !menu.contains(e.target) && e.target !== filtersButton) {
          menu.style.display = 'none';
        }
      });  
      fetchLogs();
    }
};

/*Прочие функции*/

function validateData( ip, mask, gateway) {
	

	if (ip) {
        const ipRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
        if (!ipRegex.test(ip)) {
            alert('Неверный формат IP-адреса.');
            return false;
        }
    }
	
	if (mask) {
        const maskRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
        if (!maskRegex.test(mask)) {
            alert('Неверный формат маски.');
            return false;
        }
    }
	
	if (gateway) {
		const gatewayRegex = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
	if (!gatewayRegex.test(gateway)) {
		alert('Неверный формат адреса шлюза.');
		return false;
	}
    }
    return true;
}
function saveChanges() 
{
	 document.getElementById('customModal').style.display = 'block';
}
function closeCustomModal() {
    document.getElementById('customModal').style.display = 'none';
}
function confirmChanges() {
    closeCustomModal();
    saveData(); 
}
function invaliddata() 
{
	 document.getElementById('invalid').style.display = 'block';
}
function closemodal()
{
	    document.getElementById('invalid').style.display = 'none';
}

function saveData() {
	

    const ip = document.getElementById('field2').value;
    const mask = document.getElementById('field3').value;
    const gateway = document.getElementById('field4').value;
	const pin = document.getElementById('field5').value;
	const rs485speed = document.getElementById('field4-speed').value;
	const rs485stopbit = document.getElementById('field4-stopbit').value;
	const rs485parity = document.getElementById('field4-parity').value;
	

    const updatedData = {};
	const changedValues = document.getElementById('changed-values'); //для всплывающего окна


    if (ip !== initialData.ip) {
        updatedData.ip = ip;
		changedValues.innerHTML += `<li>IP: ${ip}</li>`;
    }
    if (mask !== initialData.mask) {
        updatedData.mask = mask;
		changedValues.innerHTML += `<li>Mask: ${mask}</li>`;
    }
	
	
	if (rs485speed !== initialData.rs485speed) {
        updatedData.rs485speed = rs485speed;
		changedValues.innerHTML += `<li>rs485speed: ${rs485speed}</li>`;
    }
	
	if (rs485stopbit !== initialData.rs485stopbit) {
        updatedData.rs485stopbit = rs485stopbit;
		changedValues.innerHTML += `<li>rs485stopbit: ${rs485stopbit}</li>`;
    }
	
	if (rs485parity !== initialData.rs485parity) {
        updatedData.rs485parity = rs485parity;
		changedValues.innerHTML += `<li>rs485parity: ${rs485parity}</li>`;
    }
	
	
	
    if (gateway !== initialData.gateway) {
        updatedData.gateway = gateway;
		changedValues.innerHTML += `<li>Gateway: ${gateway}</li>`;
    }
	
	if (pin !== initialData.pin && pin.length === 6) { 
        updatedData.pin = pin;
        changedValues.innerHTML += `<li>PIN: ${pin}</li>`;
    }
	
	if (Object.keys(updatedData).length === 1 && updatedData.hasOwnProperty('pin')) {
    alert('Нет измененных данных для сохранения.');
    return;
	}

    if (Object.keys(updatedData).length === 0) {
        alert('Нет измененных данных для сохранения.');
        return;
    }

    if (!validateData(updatedData.ip, updatedData.mask, updatedData.gateway)) {
		invaliddata();
        return; 
    }
	
	const queryString = new URLSearchParams(updatedData).toString();

	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
	
		setTimeout(function() 
		{
            window.location.href = "/notion.shtml"; // Перенаправление через 2 секунды
        }, 2000);
	
	
	
}



function saveDataAdmin()
{
	const macPart1 = document.getElementById('mac1').value;
    const macPart2 = document.getElementById('mac2').value;
    const macFull = `0010A180${macPart1}${macPart2}`;
    const serial = document.getElementById('serial').value;
    const updatedData = {};
    const changedValues = document.getElementById('changed-values');
    changedValues.innerHTML = ''; 

	
	if (macPart1 || macPart2) updatedData.mac = macFull;
    if (serial) updatedData.serial = serial;

	
	    if (Object.keys(updatedData).length === 0) {
        alert('Нет измененных данных для сохранения.');
        return;
    }
	
	if (!validateDataAdmin(macFull, serial)) {
	invaliddata();
	return; 
    }
	
	const queryString = new URLSearchParams(updatedData).toString();

	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
	
		setTimeout(function() 
		{
            window.location.href = "/notion.shtml"; // Перенаправление через 2 секунды
        }, 2000);
}



function setrelay() 
{
	const queryString = "relay";
 	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
}


function test() 
{
	const queryString = "test";
 	fetch(`/save?${queryString}`, 
	{
		method: 'GET',
		headers: 
		{
			'Content-Type': 'application/json'
		}
	})
}



const mac1Element = document.getElementById('mac1');
if (mac1Element) {
  mac1Element.addEventListener('input', function() {
    if (this.value.length === 2) {
      document.getElementById('mac2').focus();
    }
  });
}

const mac2Element = document.getElementById('mac2');
if (mac2Element) {
  mac2Element.addEventListener('input', function() {
    if (this.value.length === 2) {
      document.getElementById('serial').focus();
    }
  });
}

function validateDataAdmin(macFull, serial) {
    // Проверка MAC-адреса
    if (macFull) {
        const macRegex = /^[0-9A-Fa-f]{12}$/;
        if (!macRegex.test(macFull)) {
            alert('Неверный формат MAC-адреса.');
            return false;
        }
    }

    // Проверка серийного номера: должно быть 6 цифр
    if (serial) {
        const serialRegex = /^\d{6}$/;
        if (!serialRegex.test(serial)) {
            alert('Неверный формат серийного номера.');
            return false;
        }
    }

    return true;
}

function saveChangesAdmin() 
{
	 document.getElementById('customModal').style.display = 'block';
}

function confirmChangesAdmin() {
    closeCustomModal();
    saveDataAdmin(); 
}

  function redirectToUpdatePage() {
    window.location.href = 'update.shtml'; // Перенаправление на update.shtml
  }

function editbuttoninline() {
	window.location.href = 'update.shtml';
}

function changeButtonTextBasedOnResolution() {
	var button = document.getElementById('buttonS');
	
	const canvas = document.getElementById("chartCanvas");
	const parent_canvas = document.getElementById("uptime-sectionid");
	const parentWidth = parent_canvas.offsetWidth;
	canvas.width = parentWidth * 0.95;
	
	
	
	// Проверяем разрешение экрана
	if (window.innerWidth < 1680 || window.innerHeight < 1050) {
		button.textContent = "Обновить П.О.";
	} else {
		button.textContent = "Загрузить новую версию П.О.";
	}
    }


function showAlert(alertType) {
	const modal = document.getElementById('alertModal');
	const alertMessege = document.getElementById('alertMessege');
		
	

	console.log("Найдено значение lert massege:", alertMessege.textContent); // Если нашли число


	//текст в зависимости от аргумента
	if (alertType === 1) {
		alertMessege.textContent = 'Устройство зарегистрировало устойчивый рост тока утечки. Это может указывать на деградацию изоляционных материалов. Рекомендуется провести технический осмотр и диагностику электросети. ';
	} else if (alertType === 2) {
		alertMessege.textContent = 'Внимание, ток утечки превысил порог предупреждения!';
	} else {
		alertMessege.textContent = 'дефолтный текст.';
	}

	modal.style.display = 'block';
	setTimeout(() => {
		modal.classList.add('active');
	}, 10);
}

function closeAlertModal() {
	const modal = document.getElementById('alertModal');
	modal.classList.remove('active');
	setTimeout(() => {
		modal.style.display = 'none';
	}, 300); // Ждем завершения анимации
}
		
		
function go_to_log(){
	window.location.href = "log.shtml";
}