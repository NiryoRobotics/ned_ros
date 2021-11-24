from CloudAPI import CloudAPI

c = CloudAPI('staging-api-rfm.niryo.com', 'PC_JUSTIN', '3s4LDMpvbarwRdqycZe23FWSWAgBNvr1')
print(c.daily_reports.ping())