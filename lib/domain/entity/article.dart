import 'package:dorvan/domain/entity/page_content.dart';

class Article extends PageContent {
  final String title;
  final String description;
  final String number;

  Article(this.title, this.description, this.number);
}
