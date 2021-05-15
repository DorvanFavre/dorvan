import 'package:dorvan/domain/entity/page_content.dart';

class Chapter extends PageContent {
  final String title;
  final String description;
  final String chapter;

  final List<PageContent> pages;

  Chapter(this.title, this.description, this.chapter, this.pages);
}
