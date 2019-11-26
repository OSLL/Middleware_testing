import matplotlib.pyplot as plt
import sys

count_qmsgs = 0

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

def main():
	send_time = []
	receive_time = []
	
	send = open(sys.argv[1], 'r')
	for line in send:
		send_time.append(list(map(int, line.split())))
	send.close()

	receive = open(sys.argv[2], 'r')
	for line in receive:
		receive_time.append(list(map(int, line.split()[0:-1])))
	receive.close()

	if len(send_time) != len(receive_time):
		print("Number of lines doesn't match")
		exit()
	for i in range(0, len(send_time)):
		if len(send_time[i]) != len(receive_time[i]):
			print(len(send_time[i]), len(receive_time[i]))
			print("Number of observations doesn't match")
			exit()
	list_counts = queue_size(send_time,receive_time)
	delay_time =[[receive_time[i][j] - send_time[i][j] for j in range(0, len(send_time[i]))] for i in range(0, len(send_time))]
	delay = []
	for i in range(0, 10):
		delay.append(delay_time[0][0:(500*(i+1))])
	delay_time = delay
	means = [sum(delay_time[i])/(1000000*len(delay_time[i])) for i in range(0, len(delay_time))]
	with open('delays.txt', 'w') as f:
		f.write(' '.join([str(i/1000000) for i in delay_time[-1]]))
	plt.boxplot(list(map(lambda x: [x[i]/1000000 for i in range(0, len(x))], delay_time)), positions=[len(d) for d in delay], widths=[200 for i in range(0, 10)], whis=[0,100])
	plt.ylabel('time, ms')
	plt.xlabel('count of messages')
	plt.show()
	plt.plot([i for i in range(1, len(delay_time[-1])+1)], list(map(lambda x: x/1000000, delay_time[-1])))
	plt.ylabel('time, ms')
	plt.xlabel('number of message')
	plt.show()
	plt.plot([i for i in range(1, len(list_counts)+1)], list_counts)
	plt.ylabel('messages in queue')
	plt.xlabel('number of sent messages')
	plt.show()

if __name__ == '__main__':
	if len(sys.argv) < 3:
		print('Missed send time file and/or receive time file')
		exit()
	main()
