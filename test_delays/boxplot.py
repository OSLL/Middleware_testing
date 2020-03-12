import matplotlib.pyplot as plt
import sys
import json

count_qmsgs = 0

def from_json(filename):
	with open(filename, 'r') as f:
		data = json.load(f)
	ids = [msg["msg"]["id"] for msg in data]
	send_time = [msg["msg"]["sent_time"] for msg in data]
	rec_time = [msg["msg"]["rec_time"] for msg in data]
	return ([ids], [send_time], [rec_time])

def from_txt(fpub, fsub):
	send_time = []
	receive_time = []
	
	send = open(sys.argv[1], 'r')
	for line in send:
		send_time.append(list(map(int, line.split())))
	send.close()

	receive = open(sys.argv[2], 'r')
	for line in receive:
		receive_time.append(list(map(int, line.split())))
	receive.close()
	return (send_time, receive_time)
	
def check_size(i,common):
	global count_qmsgs
	if common[i][1] is 'pub':
		count_qmsgs+=1
	else:
		count_qmsgs-=1
	return count_qmsgs

def queue_size(sent, recieved):
	sent_list = [[sent[0][i], 'pub'] for i in range(0, len(sent[0]))]
	recieved_list = [[recieved[0][i], 'sub'] for i in range(0, len(sent[0]))]
	common = sorted(sent_list + recieved_list, key = lambda el: el[0])
	list_ = [check_size(i, common) for i in range(0, len(common))]
	list_counts = []
	for i in range(0, len(list_)):
		if common[i][1] == 'pub':
			list_counts.append(list_[i])
	return list_counts

def main(send_time, receive_time, ids):
	if len(send_time) != len(receive_time):
		print("Number of lines doesn't match")
		exit()
	n = []
	for i in range(0, len(send_time)):
		if len(send_time[i]) != len(receive_time[i]):
			print(len(send_time[i]), len(receive_time[i]))
			print("Number of observations doesn't match")
			exit()
		n.append(len(send_time[i]))
	list_counts = queue_size(send_time,receive_time)
	delay_time =[[receive_time[i][j] - send_time[i][j] for j in range(0, len(send_time[i]))] for i in range(0, len(send_time))]
	delay = []
	for j in range(0, len(send_time)):
		delay.append([])
		for i in range(0, 10):
			k = int((i+1)*n[j]/10)
			delay[-1].append(delay_time[j][0:k])
	delay_time = delay
	#means = [[sum(delay_time[j][i])/(1000000*len(delay_time[j][i])) for i in range(0, len(delay_time[j]))] for j in range(0, len(delay_time))]
	with open('delays.txt', 'w') as f:
		for j in range(0, len(delay_time)):
			f.write(' '.join([str(i/1000000) for i in delay_time[j][-1]]))
	if j in range(0, len(delay_time)):
		data = list(map(lambda x: [x[i]/1000000 for i in range(0, len(x))], delay_time[j]))
		plt.boxplot(data, positions=[len(d) for d in delay[j]], widths=[n[j]/25 for i in range(0, 10)], whis=[0,100])
		plt.ylabel('time, ms')
		plt.xlabel('count of messages')
		plt.show()
		if ids is None:
			x = [i for i in range(1, len(delay_time[j][-1])+1)]
		else:
			x = ids[j]
		plt.plot(x, list(map(lambda x: x/1000000, delay_time[j][-1])))
		plt.ylabel('time, ms')
		plt.xlabel('number of message')
		plt.show()
		plt.plot([i for i in range(1, len(list_counts)+1)], list_counts)
		plt.ylabel('messages in queue')
		plt.xlabel('number of sent messages')
		plt.show()

if __name__ == '__main__':
	if len(sys.argv) == 1:
		print('Usage: '+sys.argv[0]+' [sendtime_fname rectime_fname | json_fname])')
		exit()
	ids = None
	if len(sys.argv) == 2:
		(ids, send_time, rec_time) = from_json(sys.argv[1])
	elif len(sys.argv) == 3:
		(send_time, rec_time) = from_txt(sys.argv[1], sys.argv[2])
	main(send_time, rec_time, ids)
